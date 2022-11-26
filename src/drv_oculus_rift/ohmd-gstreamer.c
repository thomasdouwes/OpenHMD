// Copyright 2021, Jan Schmidt <jan@centricular.com>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */
#include <strings.h>
#include <stdlib.h>
#include <gst/gst.h>
#include <gst/video/video.h>
#include <gst/app/app.h>

#include "ohmd-gstreamer.h"

struct ohmd_gst_pipeline_s {
	GstElement *pipeline;
	GstElement *muxer;

	guint bus_watch_id;
	GMainLoop *loop;
	GThread *bus_thread;

	gboolean at_eos;
	GMutex lock;
	GCond cond;

	GList *appsrcs;

	uint64_t base_ts;
};

struct ohmd_gst_video_stream_s {
	GstAppSrc *src;
	GstVideoInfo info;

	gint width;
	gint height;

	uint64_t base_ts;
};

struct ohmd_gst_debug_stream_s {
	GstAppSrc *src;
	uint64_t base_ts;
};

static gboolean
bus_handler (GstBus * bus, GstMessage * message, gpointer data)
{
	ohmd_gst_pipeline *p = (ohmd_gst_pipeline *)(data);

	switch (GST_MESSAGE_TYPE (message)) {
		case GST_MESSAGE_EOS:
			/* Make sure the waiter gets woken */
			g_mutex_lock(&p->lock);
			p->at_eos = TRUE;
			g_cond_broadcast(&p->cond);
			g_mutex_unlock(&p->lock);
			break;
		case GST_MESSAGE_ERROR:
		{
			GError *err = NULL;
			gchar *name;

			name = gst_object_get_path_string (message->src);
			gst_message_parse_error (message, &err, NULL);

			LOGE("Error from GStreamer pipeline. Element %s: %s", name, err->message);

			g_clear_error (&err);
			g_free (name);

			/* Make sure the waiter gets woken */
			g_mutex_lock(&p->lock);
			p->at_eos = TRUE;
			g_cond_broadcast(&p->cond);
			g_mutex_unlock(&p->lock);

			gst_element_set_state(p->pipeline, GST_STATE_NULL);
			break;
		}
		default:
			break;
	}

	return TRUE;
}


gpointer bus_thread (gpointer data)
{
	ohmd_gst_pipeline *pipe = (ohmd_gst_pipeline *)(data);
	GMainLoop *loop = g_main_loop_new(NULL, FALSE);

	g_mutex_lock(&pipe->lock);
	pipe->loop = loop;
	g_mutex_unlock(&pipe->lock);

	g_main_loop_run(loop);

	g_mutex_lock(&pipe->lock);
	pipe->loop = NULL;
	g_mutex_unlock(&pipe->lock);

	g_main_loop_unref(loop);

	return NULL;
}

static void ohmd_gst_pipeline_add_source(ohmd_gst_pipeline *pipe, GstAppSrc *src)
{
	g_mutex_lock(&pipe->lock);
	pipe->appsrcs = g_list_prepend(pipe->appsrcs, src);
	g_mutex_unlock(&pipe->lock);
}

ohmd_gst_pipeline *ohmd_gst_pipeline_new (const char *stream_id, uint64_t base_ts)
{
	ohmd_gst_pipeline *ret = NULL;
	const char *target_dir = g_getenv("OHMD_TRACE_DIR");
	char *path = NULL, *fname = NULL;
	GstBus *bus = NULL;
	GstElement *sink = NULL;

	if (target_dir == NULL || getenv ("OHMD_FULL_RECORDING") == NULL)
		return NULL;

	gst_init(NULL, NULL);

	ret = calloc(sizeof(ohmd_gst_pipeline), 1);

	ret->pipeline = gst_pipeline_new(NULL);
	if (ret->pipeline == NULL)
		goto fail;

	g_mutex_init(&ret->lock);
	g_cond_init(&ret->cond);

	ret->base_ts = base_ts;

	bus = gst_element_get_bus (ret->pipeline);
	ret->bus_watch_id = gst_bus_add_watch (bus, bus_handler, ret);
	gst_object_unref (bus);

	ret->muxer = gst_element_factory_make("matroskamux", NULL);
	if (ret->muxer == NULL) {
		LOGE("Failed to create matroskamux");
		goto fail;
	}
	// g_object_set(ret->muxer, "streamable", TRUE, NULL);

	gst_bin_add(GST_BIN (ret->pipeline), ret->muxer);

	sink = gst_element_factory_make("filesink", NULL);
	if (sink == NULL) {
		LOGE("Failed to create filesink");
		goto fail;
	}
	gst_bin_add(GST_BIN (ret->pipeline), sink);

	if (!gst_element_link(ret->muxer, sink)) {
		LOGE("Failed to link muxer and filesink");
		goto fail;
	}

	fname = g_strdup_printf("openhmd-%s.mkv", stream_id);
	path = g_build_path(G_DIR_SEPARATOR_S, target_dir, fname, NULL);
	g_free(fname);

	g_object_set(sink, "location", path, NULL);
	g_free(path);

	if (gst_element_set_state(ret->pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
		LOGW("Failed to set GStreamer recording pipeline to PLAYING");
	}

	ret->bus_thread = g_thread_new("GStreamer bus", bus_thread, ret);

	return ret;

fail:
	if (ret) {
		if (ret->pipeline) {
			gst_element_set_state(ret->pipeline, GST_STATE_NULL);
		}
		gst_clear_object(&ret->pipeline);
		free(ret);
	}

	return NULL;
}

static void send_eos(gpointer data, gpointer user_data) {
	gst_app_src_end_of_stream((GstAppSrc *)(data));
}

void ohmd_gst_pipeline_free(ohmd_gst_pipeline *pipe)
{
	if (pipe == NULL)
		return;

	/* Shut down the recording pipeline */
	if (pipe->pipeline) {
		LOGI("Finalising GStreamer recording");

		g_mutex_lock(&pipe->lock);
		g_list_foreach(pipe->appsrcs, send_eos, NULL);

		while (pipe->at_eos == FALSE)
				g_cond_wait(&pipe->cond, &pipe->lock);
		g_mutex_unlock(&pipe->lock);

		gst_element_set_state(pipe->pipeline, GST_STATE_NULL);
		gst_clear_object(&pipe->pipeline);
	}

	g_mutex_lock(&pipe->lock);
	if (pipe->loop != NULL)
		g_main_loop_quit (pipe->loop);
	g_mutex_unlock(&pipe->lock);

	g_thread_join(pipe->bus_thread);

	g_source_remove(pipe->bus_watch_id);

	g_mutex_clear(&pipe->lock);
	g_cond_clear(&pipe->cond);

	free(pipe);
}

ohmd_gst_video_stream *ohmd_gst_video_stream_new (ohmd_gst_pipeline *pipe, ohmd_pw_video_format format, uint16_t w, uint16_t h, uint16_t fps_n, uint16_t fps_d)
{
	ohmd_gst_video_stream *ret = NULL;
	GstElement *input_bin = NULL;
	GstPad *input_pad = NULL, *muxer_pad = NULL;
	GstCaps *caps = NULL;

	ret = calloc(sizeof(ohmd_gst_video_stream), 1);

	ret->width = w;
	ret->height = h;
	ret->base_ts = pipe->base_ts;

	input_bin = gst_parse_bin_from_description("appsrc name=src format=time ! jpegenc", TRUE, NULL);

	ret->src = GST_APP_SRC(gst_bin_get_by_name(GST_BIN(input_bin), "src"));
	caps = gst_caps_new_simple("video/x-raw",
						"format", G_TYPE_STRING, format == OHMD_PW_VIDEO_FORMAT_GRAY8 ? "GRAY8" : "RGB",
						"width", G_TYPE_INT, w,
						"height", G_TYPE_INT, h,
						"framerate", GST_TYPE_FRACTION, fps_n, fps_d,
						NULL);
	if (!gst_video_info_from_caps(&ret->info, caps)) {
		gchar *caps_str = gst_caps_to_string(caps);
		LOGE("Failed to parse video info from caps %s", caps_str);
		g_free(caps_str);
		goto fail;
	}

	g_object_set(ret->src, "caps", caps, NULL);
	gst_caps_unref(caps);

	gst_bin_add(GST_BIN(pipe->pipeline), input_bin);
	input_pad = gst_element_get_static_pad(input_bin, "src");

	muxer_pad = gst_element_get_request_pad(pipe->muxer, "video_%u");
	if (gst_pad_link(input_pad, muxer_pad) != GST_PAD_LINK_OK) {
		LOGW("Failed to link video input stream to recording pipeline");
		goto fail;
	}
	gst_object_unref(input_pad);
	gst_object_unref(muxer_pad);

	gst_element_set_state(input_bin, GST_STATE_PLAYING);

	ohmd_gst_pipeline_add_source(pipe, ret->src);

	return ret;

fail:
	if (ret) {
		gst_clear_object(&ret->src);
		free(ret);
	}

	gst_clear_object(&input_pad);
	gst_clear_object(&muxer_pad);

	return NULL;
}

void ohmd_gst_video_stream_push (ohmd_gst_video_stream *v, int64_t pts, const uint8_t *pixels)
{
	GstBuffer *buf = gst_buffer_new_allocate(NULL, v->info.size, NULL);
	GstVideoFrame frame;
	const guint8 *in = pixels;
	guint8 *out;
	guint y = 0;

	if (!gst_video_frame_map(&frame, &v->info, buf, GST_MAP_WRITE)) {
		LOGW("Could not record video frame to GStreamer");
		gst_buffer_unref(buf);
		return;
	}

	out = frame.data[0];
	for (y = 0; y < v->height; y++) {
		memcpy (out, in, v->width);
		out += GST_VIDEO_INFO_PLANE_STRIDE (&v->info, 0);
		in += v->width;
	}

	gst_video_frame_unmap(&frame);

	GST_BUFFER_PTS(buf) = pts - v->base_ts;

	if (gst_app_src_push_buffer(v->src, buf) != GST_FLOW_OK) {
		LOGW("Failed to push buffer to GStreamer recording pipeline");
	}
}

void ohmd_gst_video_stream_free (ohmd_gst_video_stream *v)
{
	gst_app_src_end_of_stream(v->src);
	free(v);
}

ohmd_gst_debug_stream *ohmd_gst_debug_stream_new (ohmd_gst_pipeline *pipe)
{
	ohmd_gst_debug_stream *ret = NULL;
	GstPad *input_pad = NULL, *muxer_pad = NULL;
	GstCaps *caps = NULL;

	ret = calloc(sizeof(ohmd_gst_debug_stream), 1);

	ret->base_ts = pipe->base_ts;
	ret->src = GST_APP_SRC(gst_element_factory_make("appsrc", NULL));
	if (ret->src == NULL)
		goto fail;
	gst_util_set_object_arg(G_OBJECT(ret->src), "format", "time");

	caps = gst_caps_new_simple("text/x-raw", "format", G_TYPE_STRING, "utf8", NULL);
	g_object_set(ret->src, "caps", caps, NULL);
	gst_caps_unref(caps);

	gst_bin_add(GST_BIN(pipe->pipeline), GST_ELEMENT_CAST(ret->src));

	input_pad = gst_element_get_static_pad(GST_ELEMENT_CAST(ret->src), "src");
	muxer_pad = gst_element_get_request_pad(pipe->muxer, "subtitle_%u");

	if (gst_pad_link(input_pad, muxer_pad) != GST_PAD_LINK_OK) {
		LOGW("Failed to link subtitle input stream to recording pipeline");
		goto fail;
	}
	gst_object_unref(input_pad);
	gst_object_unref(muxer_pad);

	gst_element_set_state(GST_ELEMENT(ret->src), GST_STATE_PLAYING);

	ohmd_gst_pipeline_add_source(pipe, ret->src);

	return ret;

fail:
	if (ret) {
		gst_clear_object(&ret->src);
		free(ret);
	}

	gst_clear_object(&input_pad);
	gst_clear_object(&muxer_pad);

	return NULL;
		return NULL;
}

void ohmd_gst_debug_stream_push (ohmd_gst_debug_stream *s, int64_t pts, const char *debug_str)
{
	GstBuffer *buf;
	gchar *buf_content = g_strdup(debug_str);

	buf = gst_buffer_new_wrapped(buf_content, strlen(buf_content) + 1);
	GST_BUFFER_PTS(buf) = pts - s->base_ts;

	if (gst_app_src_push_buffer(s->src, buf) != GST_FLOW_OK) {
		LOGW("Failed to push buffer to GStreamer recording pipeline");
	}
}

void ohmd_gst_debug_stream_free (ohmd_gst_debug_stream *s)
{
	gst_app_src_end_of_stream(s->src);
	free(s);
}


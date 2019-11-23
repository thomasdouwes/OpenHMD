// Copyright 2019, Google.

#include "softposit.h"

using namespace std;

#define SPDEBUG_ALL 0
#define SPDEBUG_INIT (SPDEBUG_ALL && 0)
#define SPDEBUG_L (SPDEBUG_ALL && 0)
#define SPDEBUG_ASSIGN (SPDEBUG_ALL && 0)
#define SPDEBUG_ROTTRANS (SPDEBUG_ALL && 0)
#define SPDEBUG_LOOP (SPDEBUG_ALL && 0)
#define SPDEBUG_DONE (SPDEBUG_ALL && 0)

Object* softposit_new_object(std::vector<cv::Vec3d> points) {
  Object* obj = new Object; //(Object*)calloc(1, sizeof(Object));
  obj->points = points;
  // pose vectors TODO: random or initial pose?
  obj->pose1 = cv::Vec4d(0.0f);
  obj->pose2 = cv::Vec4d(0.0f); 
  obj->translation = cv::Vec3d(0.0f);
  obj->rotation = cv::Mat(3, 3, CV_64F, cv::Scalar(0.0f));

  return obj;
}

void softposit_free_object(Object *obj) {
  // obj->points.std::vector::~vector();
  // obj->pose1.cv::Vec4f::~Vec4f();
  // obj->pose2.cv::Vec4f::~Vec4f();
  // obj->translation.cv::Vec3f::~Vec3f();
  // obj->rotation.cv::Mat::~Mat();
  delete obj;
}


void assign_resize(assign_mat *mat, size_t width, size_t height);

assign_mat assign_mat_new(size_t width, size_t height) {
  assign_mat mat;
  mat.width = 0;
  mat.height = 0;
  mat.data_len = 0;
  mat.data = NULL;
  mat.assign_sum = 0;
  mat.assignsq_sum = 0;
  mat.slack_sum = 0;
  assign_resize(&mat, width, height);
  return mat;
}

void assign_mat_free(assign_mat *mat) {
  free(mat->data);
  mat->data = NULL;
  mat->data_len = 0;
}

void assign_resize(assign_mat *mat, size_t width, size_t height) {
  size_t new_len = width * height + 
      /* slack: */ width + height + 
      /* sums: */ width + height + 
      /* extra: */ max(width, height);
  if (new_len > mat->data_len) {
    assign_mat_free(mat);
    mat->data = (double*)calloc(new_len, sizeof(double));
    mat->data_len = new_len;
  }

  mat->width = width;
  mat->height = height;
}

void assign_assert_xy(assign_mat *mat, size_t x, size_t y) {
  assert(x < mat->width && y < mat->height);
}

size_t _assign_idx(assign_mat *mat, size_t x, size_t y) {
  return y * mat->width + x;
}

size_t _assign_colslack(assign_mat *mat, size_t x) {
  return mat->width * mat->height + x;
}

double assign_colslack(assign_mat *mat, size_t x) {
  return mat->data[_assign_colslack(mat, x)];
}

size_t _assign_rowslack(assign_mat *mat, size_t y) {
  return mat->width * mat->height + mat->width + y;
}

double assign_rowslack(assign_mat *mat, size_t y) {
  return mat->data[_assign_rowslack(mat, y)];
}

size_t _assign_colsum(assign_mat *mat, size_t x) {
  assert(x < mat->width);
  return mat->width * mat->height + mat->width + mat->height + x;
}

double assign_colsum(assign_mat *mat, size_t x) {
  return mat->data[_assign_colsum(mat, x)];
}

size_t _assign_rowsum(assign_mat *mat, size_t y) {
  assert(y < mat->height);
  return mat->width * mat->height + mat->width + mat->height + mat->width + y;
}

size_t _assign_extra(assign_mat *mat, size_t i) {
  assert(i < max(mat->width, mat->height));
  return mat->width * mat->height + mat->width + mat->height + mat->width + mat->height + i;
}

double assign_rowsum(assign_mat *mat, size_t y) {
  return mat->data[_assign_rowsum(mat, y)];
}

double _assign_set_data_diff(assign_mat *mat, size_t idx, double val) {
  assert(idx < mat->data_len);
  double* ptr = &mat->data[idx];
  double diff = val - *ptr;
  if (idx < mat->width * mat->height) {
    mat->assign_sum += diff;
    mat->assignsq_sum += val * val - *ptr * *ptr;
  } else
    mat->slack_sum += diff;
  *ptr = val;
  return diff;
}


double assign_get(assign_mat *mat, size_t x, size_t y) {
  assign_assert_xy(mat, x, y);
  return mat->data[_assign_idx(mat, x, y)];
}

void assign_set(assign_mat *mat, size_t x, size_t y, double val) {
  assign_assert_xy(mat, x, y);
  double diff = _assign_set_data_diff(mat, _assign_idx(mat, x, y), val);
  mat->data[_assign_colsum(mat, x)] += diff;
  mat->data[_assign_rowsum(mat, y)] += diff;
}

double assign_mult(assign_mat *mat, size_t x, size_t y, double mult) {
  double val = assign_get(mat, x, y) * mult;
  assign_set(mat, x, y, val);
  return val;
}

void assign_set_colslack(assign_mat *mat, size_t x, double val) {
    mat->data[_assign_colsum(mat, x)] += _assign_set_data_diff(mat, _assign_colslack(mat, x), val);
}

void assign_set_rowslack(assign_mat *mat, size_t y, double val) {
    mat->data[_assign_rowsum(mat, y)] += _assign_set_data_diff(mat, _assign_rowslack(mat, y), val);
}

double assign_mult_colslack(assign_mat *mat, size_t x, double mult) {
  double val = assign_colslack(mat, x) * mult;
  assign_set_colslack(mat, x, val);
  return val;
}
double assign_mult_rowslack(assign_mat *mat, size_t y, double mult) {
  double val = assign_rowslack(mat, y) * mult;
  assign_set_rowslack(mat, y, val);
  return val;
}

void assign_set_all_slack(assign_mat *mat, double val) {
  for (size_t x = 0; x < mat->width; x++) {
    assign_set_colslack(mat, x, val);
  }
  for (size_t y = 0; y < mat->height; y++) {
    assign_set_rowslack(mat, y, val);
  }
}

void assign_print(assign_mat *mat) {
  printf("sum: %f  sqsum: %f  slack_sum: %f\n", mat->assign_sum, mat->assignsq_sum, mat->slack_sum);
  for (size_t y = 0; y < mat->height; y++) {
    for (size_t x = 0; x < mat->width; x++) {
      printf("%.2f ", assign_get(mat, x, y));
    }
    printf("%.2f %.2f\n", assign_rowslack(mat, y), assign_rowsum(mat, y));
  }
  for (size_t x = 0; x < mat->width; x++) {
    printf("%.2f ", assign_colslack(mat, x));
  }
  printf("\n");
  for (size_t x = 0; x < mat->width; x++) {
    printf("%.2f ", assign_colsum(mat, x));
  }
  printf("\n");
}

assign_mat *assign_normalize(assign_mat *assign2, assign_mat *assign1, double small, double max_slack) {
  size_t x, y;
  // Sinkhorn's method?
  double assign_norm = 0, last_assign_norm = 0;
  assign_mat *assign = assign2;
  assign_mat *assign_prev = assign1;
  do {
    last_assign_norm = assign_norm;
    assign_norm = 0;

    // save a copy of what rowsum is before normalizing
    // for (y = 0; y < assign->height; y++) {
    //   assign->data[_assign_extra(assign, y)] = assign_rowsum(assign_prev, y);
    // }

    // normalize nonslack rows
    for (x = 0; x < assign->width; x++) {
      double inv_sum = 1.0 / assign_colsum(assign_prev, x);
      // printf("assign_colsum(assign_prev, x): %f\n", assign_colsum(assign_prev, x));
      // printf("inv_sum: %f\n", inv_sum);
      for (y = 0; y < assign->height; y++) {
        assign_set(assign, x, y, assign_get(assign_prev, x, y) * inv_sum);
      }

      double val = assign_colslack(assign_prev, x) * inv_sum;
      assign_set_colslack(assign, x, min(val, max_slack));
    }
    // printf("assign.25:\n"); assign_print(assign);

    // copy slack
    // All the other values have been scaled except for the rowslacks.
    // So try to scale them appropriately
    for (y = 0; y < assign->height; y++) {
      double old_slack = assign_rowslack(assign_prev, y);
      assign_set_rowslack(assign, y, old_slack);
      double sum = assign_rowsum(assign, y);
      double old_sum = assign_rowsum(assign_prev, y);
      double slack = old_slack;
      if (abs(old_sum - old_slack) > 0.001)
        slack = old_slack * (sum - old_slack) / (old_sum - old_slack);
      // printf("scale slack: %f %f %f %f\n", old_sum, sum, old_slack, slack);
      assign_set_rowslack(assign, y, min(slack, max_slack));
    }

    // printf("assign.5:\n"); assign_print(assign);

    // save a copy of what colsum is before normalizing
    for (x = 0; x < assign->width; x++) {
      assign->data[_assign_extra(assign, x)] = assign_colsum(assign, x);
    }
    // normalize nonslack columns
    for (y = 0; y < assign->height; y++) {
      double inv_sum = 1.0 / assign_rowsum(assign, y);
      // printf("objects_sum[y]: %f\n", objects_sum[y]);
      // printf("inv_sum: %f\n", inv_sum);
      for (x = 0; x < assign->width; x++) {
        double val = assign_mult(assign, x, y, inv_sum);
        assign_norm += pow(val - assign_get(assign_prev, x, y), 2.0);
      }

      double val = assign_rowslack(assign, y);
      val *= inv_sum;
      val = min(val, max_slack);
      assign_set_rowslack(assign, y, val);
      assign_norm += pow(val - assign_rowslack(assign_prev, y), 2.0);
    }

    // printf("assign.75:\n"); assign_print(assign);

    // All the other values have been scaled except for the colslacks.
    // So try to scale them appropriately
    for (x = 0; x < assign->width; x++) {
      double old_slack = assign_colslack(assign, x);
      double sum = assign_colsum(assign, x);
      double old_sum = assign->data[_assign_extra(assign, x)];
      double slack = old_slack;
      if (abs(old_sum - old_slack) > 0.001)
        slack = old_slack * (sum - old_slack) / (old_sum - old_slack);
      slack = min(slack, max_slack);
      // printf("scale slack: %f %f %f %f\n", old_sum, sum, old_slack, slack);
      assign_set_colslack(assign, x, slack);

      assign_norm += pow(slack - assign_colslack(assign_prev, x), 2.0);
    }

    if (SPDEBUG_ASSIGN) printf("assign_norm: %f  sum: %f  sqsum: %f  slack_sum: %f\n", assign_norm, assign->assign_sum, assign->assignsq_sum, assign->slack_sum);
    // printf("assign:\n"); assign_print(assign);

    // swap assign and assign_prev
    assign = assign_prev;
    assign_prev = assign == assign1? assign2 : assign1;
  } while (assign_norm > small && abs(last_assign_norm - assign_norm) > 0.001); // what is small here?

  // printf("assign_prev:\n"); assign_print(assign);
  // printf("assign:\n"); assign_print(assign_prev);

  return assign_prev;
}


cv::Vec4d v3_to_v4(cv::Vec3d v, float w) {
  return cv::Vec4d(v[0], v[1], v[2], w);
}

void print_vec(const char* name, cv::Vec4d v) {
  printf("%s: [%f, %f, %f, %f]\n", name, v[0], v[1], v[2], v[3]);
}
void print_vec(const char* name, cv::Vec3d v) {
  printf("%s: [%f, %f, %f]\n", name, v[0], v[1], v[2]);
}
void print_vector(const char* name, std::vector<double> v) {
  printf("%s: [", name);
  for (size_t i = 0; i < v.size(); i++) {
      printf("% .2e ", v[i]);
  }
    printf("]\n");
}
void print_mat(const char* name, cv::Mat* m) {
  printf("%s: \n", name);
  for (int i = 0; i < m->rows; i++) {
    printf("    ");
    for (int j = 0; j < m->cols; j++) {
      printf("% .2e ", m->at<double>(i, j));
    }
    printf("\n");
  }
}

softposit_data* softposit_new() {
  softposit_data *data = new softposit_data;

  data->objects = std::vector<Object*>(0);
  data->correction = std::vector<double>(0);

  data->beta_final = 0.5;
  data->beta_update = 1.05;
  data->small = 0.00001;
  data->focal_length = 1.0; // ?
  data->alpha = 10; // TODO: This is often computed based on noise in the image.
  data->num_object_points = 0;

  data->assign1 = assign_mat_new(0, 0);
  data->assign2 = assign_mat_new(0, 0);

  return data;
}

void softposit_free(softposit_data* data) {
  assign_mat_free(&data->assign1);
  assign_mat_free(&data->assign2);
  delete data;
}

void softposit_add_object(softposit_data* data, Object *obj) {
  // printf("add_object %ld\n", data->objects.size());
  data->objects.push_back(obj);
  data->num_object_points += obj->points.size();
  if (data->num_object_points > 0) {
    // printf("add_object %ld %ld\n", data->num_object_points, obj->points.size());
    data->correction.resize(data->num_object_points);
  }
}

assign_mat *softposit_one_setup(
  softposit_data *data,
  const std::vector<cv::Point2f> &image_points,
  double slack,
  double beta
);
void softposit_one(
  softposit_data *data,
  const std::vector<cv::Point2f> &image_points,
  double slack,
  double beta
);

void softposit_init(
  softposit_data *data,
  const std::vector<cv::Point2f> &image_points,
  double slack
) {
  size_t i, o, j, k;
  Object *obj;

  double x, y;
  double xmin = image_points[0].x, xmax = image_points[0].x;
  double ymin = image_points[0].y, ymax = image_points[0].y;

  assign_resize(&data->assign1, image_points.size(), data->num_object_points);
  assign_resize(&data->assign2, image_points.size(), data->num_object_points);

  for (j = 0; j < image_points.size(); j++) {
    x = image_points[j].x;
    y = image_points[j].y;
    if (SPDEBUG_INIT) printf("%ld imgpoint: %f %f \n", j, x, y);
    xmin = min(x, xmin);
    xmax = max(x, xmax);
    ymin = min(y, ymin);
    ymax = max(y, ymax);
  }
  if (SPDEBUG_INIT) printf("x: min: %f  max: %f\n", xmin, xmax);
  if (SPDEBUG_INIT) printf("y: min: %f  max: %f\n", ymin, ymax);

  double maxlen;

  cv::Vec4d best_pose1;
  cv::Vec4d best_pose2;
  double best_sqsum = 0;

  // initialize correction
  for (k = 0; k < data->num_object_points; k++) {
    data->correction[k] = 1;
  }

  for (o = 0, k = 0; o < data->objects.size(); o++) {
    obj = data->objects[o];

    maxlen = 0;

    for (i = 0; i < obj->points.size(); i++, k++) {
      if (SPDEBUG_INIT) printf("%ld ", k);
      if (SPDEBUG_INIT) print_vec("objpoint", v3_to_v4(obj->points[i], 1));
      double len = cv::norm(obj->points[i]);
      maxlen = max(len, maxlen);
    }

    if (SPDEBUG_INIT) printf("obj %ld maxlen: %f\n", o, maxlen);

    double poselen =  1/ maxlen;

    // TODO: Calculate pose vectors from initial rot/trans
    double xsize = poselen / ((xmax - xmin) / 2);
    double ysize = poselen / ((ymax - ymin) / 2);
    if (SPDEBUG_INIT) printf("xsize: %f  ysize: %f\n", xsize, ysize);

    // try a bunch of different random poses, see which one matches best
    for (size_t p = 0; p < 500; p++) {
      // there's no guarantee that these will be orthogonal
      // a better randomization that makes orthogonal vectors might help
      randu(obj->pose1, cv::Scalar(-1.0), cv::Scalar(1.0));
      randu(obj->pose2, cv::Scalar(-1.0), cv::Scalar(1.0));

      // try constraining the pose to be mostly upright?
      obj->pose1[1] /= 1000.0;
      obj->pose2[0] /= 10.0;
      obj->pose2[1] /= 10.0;

      obj->pose1[3] = 0;
      obj->pose2[3] = 0;
      obj->pose1 *= xsize / cv::norm(obj->pose1);
      obj->pose2 *= ysize / cv::norm(obj->pose2);
      obj->pose1[3] = ((xmax + xmin) / 2.0 + 1.0);
      obj->pose2[3] = ((ymax + ymin) / 2.0 + 1.0);
      if (SPDEBUG_INIT) print_vec("init pose1", obj->pose1);
      if (SPDEBUG_INIT) print_vec("init pose2", obj->pose2);

      // initialize correction
      for (k = 0; k < data->num_object_points; k++) {
        data->correction[k] = 1;
      }
      
      // run a few iterations of the loop just for fun
      softposit_one(data, image_points, slack, 0.0004);
      softposit_one(data, image_points, slack, 0.004);
      softposit_one(data, image_points, slack, 0.04);
      assign_mat *assign = softposit_one_setup(data, image_points, slack, 0.4);
      // printf("random assign:  sum: %f  sqsum: %f  slacksum: %f\n", assign->assign_sum, assign->assignsq_sum, assign->slack_sum);

      // using sqsum is an attempt to approximate how specific the assign matrix is.
      // values should be between 0 and 1 (TODO: they aren't always, why?), so squaring
      // the values should make values near 1 stay the same, while values closer to 0
      // go way closer to 0. A sqsum value near the number of image points is probably
      // the best goal. 
      if (assign->assignsq_sum > best_sqsum) {
        best_sqsum = assign->assignsq_sum;
        best_pose1 = obj->pose1;
        best_pose2 = obj->pose2;
      }
    }

    printf("best_sqsum: %f\n", best_sqsum);

    obj->pose1 = best_pose1;
    obj->pose2 = best_pose2;
  }
}

void softposit_squared_dists(
  softposit_data *data,
  const std::vector<cv::Point2f> &image_points,
  double slack,
  double beta
) {
  size_t i, o, j, k;
  Object *obj;

  for (o = 0, k = 0; o < data->objects.size(); o++) {
    obj = data->objects[o];

    // print_vec("pose1", obj->pose1);
    // print_vec("pose2", obj->pose2);
    for (i = 0; i < obj->points.size(); i++, k++) {
      // printf("%ld %ld ", i, k);
      // print_vec("point", v3_to_v4(obj->points[i], 1));
      double dot1 = obj->pose1.dot(v3_to_v4(obj->points[i], 1));
      double dot2 = obj->pose2.dot(v3_to_v4(obj->points[i], 1));
      // printf("dot: %f %f\n", dot1, dot2);
      if (isnan(dot1)) abort();

      // printf("distsq:");
      for (j = 0; j < image_points.size(); j++) {
        
        // compute squared distances
        double distsq = 
          pow(dot1 - data->correction[k] * image_points[j].x, 2.0) + 
          pow(dot2 - data->correction[k] * image_points[j].y, 2.0);
        
        // distsq += (rand() % 100) / 1000.0;
        
        // this is the formula from the paper, but if you use it then the
        // assign matrix never seems to try to assign any blobs to leds,
        // if you sum the squares of all the values, it is always near 0.
        // double val = slack * exp(-beta * (distsq - data->alpha));

        // This is a formula I made up. It forces the values to spread out
        // more so that the assign matrix will actually choose something
        // instead of being wishy-washy.
        double val = beta / (distsq * data->alpha);
        // val = pow(val, 1 + beta*2);

        // printf("  %f,%f", distsq, val);
        // if (abs(val) < 0.000001)
        //   val = (random()%1000) / 1000000.0;
        assign_set(&data->assign1, j, k, val);
      }
      // printf("\n");
    }
  }
}

cv::Mat softposit_compute_L_inv(
  softposit_data *data,
  assign_mat *assign
) {
  size_t i, o, k;
  Object *obj;

  // compute 4x4 matrix L TODO: Better name?
  cv::Mat L(4, 4, CV_64F, cv::Scalar(0));
  for (o = 0, k = 0; o < data->objects.size(); o++) {
    obj = data->objects[o];
    for (i = 0; i < obj->points.size(); i++, k++) {
      // printf("L 1...");
      cv::Mat a = cv::Mat(v3_to_v4(obj->points[i], 1));
      // print_mat("L a", &a);
      cv::Mat tmp = cv::Mat(a * a.t());
      // print_mat("L tmp", &tmp);
      // printf("rowsum[k]-rowslack[k]: %f\n", assign_rowsum(assign, k)-assign_rowslack(assign, k));
      L += (assign_rowsum(assign, k)-assign_rowslack(assign, k)) * tmp;
      // printf("L 4...");
    }
  }

  // L /= 10000.0;
  if (SPDEBUG_L) print_mat("L", &L);

  cv::Mat L_inv = L.inv();
  if (SPDEBUG_L) print_mat("L_inv", &L_inv);

  return L_inv;
}

bool randomize_vec3_if_near_zero(cv::Vec4d &v) {
  if (abs(v[0]) + abs(v[1]) + abs(v[2]) < 0.001) {
    printf("randomizing vec3\n");
    v[0] = rand()%1000 / 100.0;
    v[1] = rand()%1000 / 100.0;
    v[2] = rand()%1000 / 100.0;
    v[3] = rand()%1000 / 100.0;
    return true;
  }
  return false;
}

void softposit_update_pose_vectors(
  softposit_data *data,
  const std::vector<cv::Point2f> &image_points,
  assign_mat *assign,
  cv::Mat &L_inv,
  size_t &k,
  Object *obj
) {
  size_t i, j;
  cv::Vec4d tmp;

  obj->pose1 = cv::Scalar(0);
  obj->pose2 = cv::Scalar(0);

  for (i = 0; i < obj->points.size(); i++, k++) {
    for (j = 0; j < image_points.size(); j++) {
      tmp = assign_get(assign, j, k) * data->correction[k] * v3_to_v4(obj->points[i], 1);
      // print_vec("pose tmp", tmp);
      obj->pose1 = cv::Vec4d(obj->pose1 + tmp * image_points[j].x);
      // print_vec("pose1", obj->pose1);
      obj->pose2 = cv::Vec4d(obj->pose2 + tmp * image_points[j].y);
    }
  }
  
  obj->pose1 = cv::Mat(L_inv * obj->pose1);
  obj->pose2 = cv::Mat(L_inv * obj->pose2);
}

void softposit_compute_rot_trans(
  softposit_data *data,
  Object *obj
) {
  double s_inv = 1.0/sqrt(
    sqrt(pow(obj->pose1[0], 2.0) + pow(obj->pose1[1], 2.0) + pow(obj->pose1[2], 2.0)) *
    sqrt(pow(obj->pose2[0], 2.0) + pow(obj->pose2[1], 2.0) + pow(obj->pose2[2], 2.0))
  );

  if (SPDEBUG_ROTTRANS) printf("s_inv: %f\n", s_inv);
  if (isinf(s_inv)) abort();

  cv::Vec4d rtmp = obj->pose1 * s_inv;
  // print_vec("pose1 * s_inv", rtmp);
  obj->rotation.at<double>(0, 0) = rtmp[0];
  obj->rotation.at<double>(0, 1) = rtmp[1];
  obj->rotation.at<double>(0, 2) = rtmp[2];
  rtmp = obj->pose2 * s_inv;
  // print_vec("pose2 * s_inv", rtmp);
  obj->rotation.at<double>(1, 0) = rtmp[0];
  obj->rotation.at<double>(1, 1) = rtmp[1];
  obj->rotation.at<double>(1, 2) = rtmp[2];
  cv::Vec3d r3tmp = cv::Vec3d(obj->rotation.row(0)).cross(cv::Vec3d(obj->rotation.row(1)));
  // print_vec("r1 x r2", r3tmp);
  obj->rotation.at<double>(2, 0) = r3tmp[0];
  obj->rotation.at<double>(2, 1) = r3tmp[1];
  obj->rotation.at<double>(2, 2) = r3tmp[2]; // R3
  if (SPDEBUG_ROTTRANS) print_mat("rot", &obj->rotation);

  // printf("softposit compute trans\n");

  obj->translation[0] = obj->pose1[3] * s_inv; // T1
  obj->translation[1] = obj->pose2[3] * s_inv; // T2
  obj->translation[2] = data->focal_length * s_inv; // T3
  if (SPDEBUG_ROTTRANS) print_vec("trans", obj->translation);
}

void softposit_compute_correction(
  softposit_data *data
) {
  size_t i, o, k;
  Object *obj;

  for (o = 0, k = 0; o < data->objects.size(); o++) {
    obj = data->objects[o];
    for (i = 0; i < obj->points.size(); i++, k++) {
      data->correction[k] = cv::Vec3d(obj->rotation.row(2)).dot(obj->points[i])/obj->translation[2] + 1;
    }
  }
}

assign_mat *softposit_one_setup(
  softposit_data *data,
  const std::vector<cv::Point2f> &image_points,
  double slack,
  double beta
) {
    // is this a good idea?
    assign_set_all_slack(&data->assign1, slack /*min(slack, beta * 2)*/);

    softposit_squared_dists(data, image_points, slack, beta);

    // printf("assign1:\n"); assign_print(&data->assign1);

    return assign_normalize(&data->assign2, &data->assign1, data->small, 1 /*beta * 2*/);
}

void softposit_one(
  softposit_data *data,
  const std::vector<cv::Point2f> &image_points,
  double slack,
  double beta
) {
    assign_mat *assign = softposit_one_setup(data, image_points, slack, beta);

    cv::Mat L_inv = softposit_compute_L_inv(data, assign);

    bool some_zero = false;

    // compute new Q1 and Q2
    for (size_t o = 0, k = 0; o < data->objects.size(); o++) {
      Object *obj = data->objects[o];

      softposit_update_pose_vectors(data, image_points, assign, L_inv, k, obj);

      // try to kick pose vectors out of the 0 rut?
      // bool z1 = randomize_vec3_if_near_zero(obj->pose1);
      // bool z2 = randomize_vec3_if_near_zero(obj->pose2);
      if (SPDEBUG_LOOP) print_vec("pose1", obj->pose1);
      if (SPDEBUG_LOOP) print_vec("pose2", obj->pose2);

      // if (z1 || z2) {
      //   some_zero = true;
      //   continue;
      // }
      

      softposit_compute_rot_trans(data, obj);

    }

    if (!some_zero) {

      softposit_compute_correction(data);
      if (SPDEBUG_LOOP) print_vector("correction", data->correction);
    }
}

void softposit(
  softposit_data *data,
  const std::vector<cv::Point2f> &image_points
) {
  // this can be larger with a guessed initial pose
  double beta = 0.0004; // anealing amount? TODO: Better name
  double beta_final = data->beta_final;

  size_t num_image_points = image_points.size();
  size_t num_object_points = data->num_object_points;

  size_t size = num_object_points > num_image_points? num_object_points : num_image_points;
  double slack = 1.0/(size + 1.0); // better name?
  if (SPDEBUG_ROTTRANS) printf("slack: %f\n", slack);

  softposit_init(data, image_points, slack);


  // printf("softposit init\n");
  // initialize slack elements of assign
  assign_set_all_slack(&data->assign1, slack);
  assign_set_all_slack(&data->assign2, slack);

  if (SPDEBUG_ROTTRANS) {
    printf("assign1:\n"); assign_print(&data->assign1);
    printf("assign2:\n"); assign_print(&data->assign2);
  }


  // deterministic annealing loop
  while (beta < beta_final) {

    if (SPDEBUG_LOOP) printf("\n\nsoftposit loop start beta: %f\n", beta);

    softposit_one(data, image_points, slack, beta);

    beta *= data->beta_update;
    // break;
  }

  if (SPDEBUG_DONE) {
    printf("softposit done\n");
    printf("assign1:\n"); assign_print(&data->assign1);
  }
}

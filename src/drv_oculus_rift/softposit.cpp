// Copyright 2019, Google.

#include "softposit.h"

using namespace std;

#define SPDEBUG_ALL 1
#define SPDEBUG_INIT (SPDEBUG_ALL && 1)
#define SPDEBUG_DISTSQ (SPDEBUG_ALL && 1)
#define SPDEBUG_DISTSQ_V (SPDEBUG_ALL && 0)
#define SPDEBUG_L (SPDEBUG_ALL && 1)
#define SPDEBUG_ASSIGN (SPDEBUG_ALL && 1)
#define SPDEBUG_ROTTRANS (SPDEBUG_ALL && 0)
#define SPDEBUG_LOOP (SPDEBUG_ALL && 1)
#define SPDEBUG_KBWAIT (SPDEBUG_ALL && 0)
#define SPDEBUG_DONE (SPDEBUG_ALL && 1)

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
      /* extra: */ width + height;
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

double assign_get(assign_mat *mat, size_t x, size_t y) {
  assign_assert_xy(mat, x, y);
  return mat->data[_assign_idx(mat, x, y)];
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
  // return mat->data[_assign_colsum(mat, x)];
  double sum = 0;
  for (size_t y = 0; y < mat->height; y++) {
    sum += assign_get(mat, x, y);
  }
  sum += assign_colslack(mat, x);
  return sum;
}

size_t _assign_rowsum(assign_mat *mat, size_t y) {
  assert(y < mat->height);
  return mat->width * mat->height + mat->width + mat->height + mat->width + y;
}

size_t _assign_extra(assign_mat *mat, size_t i) {
  assert(i < mat->width + mat->height);
  return mat->width * mat->height + mat->width + mat->height + mat->width + mat->height + i;
}

double assign_extra(assign_mat *mat, size_t i) {
  return mat->data[_assign_extra(mat, i)];
}
void assign_set_extra(assign_mat *mat, size_t i, double val) {
    mat->data[_assign_extra(mat, i)] = val;
}

double assign_rowsum(assign_mat *mat, size_t y) {
  // return mat->data[_assign_rowsum(mat, y)];
  double sum = 0;
  for (size_t x = 0; x < mat->width; x++) {
    sum += assign_get(mat, x, y);
  }
  sum += assign_rowslack(mat, y);
  return sum;
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

// return the squared sum of normal elemnts in the assign matrix
double assign_sqsum(assign_mat *assign) {
  // return assign->assignsq_sum;
  double sum = 0.0;
  size_t x, y;

  for (y = 0; y < assign->height; y++) {
    for (x = 0; x < assign->width; x++) {
      sum += pow(assign_get(assign, x, y), 2.0);
    }
  }

  return sum;
}

void assign_print(assign_mat *mat) {
  for (size_t y = 0; y < mat->height; y++) {
    for (size_t x = 0; x < mat->width; x++) {
      printf("%.5f ", assign_get(mat, x, y));
    }
    printf("%.5f %.2f\n", assign_rowslack(mat, y), assign_rowsum(mat, y));
  }
  for (size_t x = 0; x < mat->width; x++) {
    printf("%.2f ", assign_colslack(mat, x));
  }
  printf("\n");
  for (size_t x = 0; x < mat->width; x++) {
    printf("%.2f ", assign_colsum(mat, x));
  }
  printf("sqsum: %f\n", assign_sqsum(mat));
  printf("\n");
}

// When normalizing, say, all the rows, all the main values and all the rowslack values get
// scaled, leaving the colslack values out. This rescales those values based on how the sum changed
// so that the colslack values stay somewhat proportional to the column's maximum value.
void assign_rescale_rowslack(assign_mat *assign, assign_mat *assign_prev, double max_slack) {
  size_t y;
  
  for (y = 0; y < assign->height; y++) {
    double old_slack = assign_rowslack(assign_prev, y);
    assign_set_rowslack(assign, y, old_slack);
    double sum = assign_rowsum(assign, y);
    double old_sum = assign->data[_assign_extra(assign, y)];
    double slack = old_slack;
    if (abs(old_sum - old_slack) > 0.00001) // avoid divide by 0 if slack is the only value in the row
      slack = old_slack * (sum - old_slack) / (old_sum - old_slack);
    // printf("scale slack: %f %f %f %f\n", old_sum, sum, old_slack, slack);
    assign_set_rowslack(assign, y, slack);
  }
}

void assign_rescale_colslack(assign_mat *assign, assign_mat *assign_prev, double max_slack) {
  size_t x;
  
  for (x = 0; x < assign->width; x++) {
      double old_slack = assign_colslack(assign_prev, x);
      double sum = assign_colsum(assign, x);
      double old_sum = assign->data[_assign_extra(assign, x)];
      double slack = old_slack;
      if (abs(old_sum - old_slack) > 0.0001)
        slack = old_slack * (sum - old_slack) / (old_sum - old_slack);
      // printf("scale slack: %f %f %f %f\n", old_sum, sum, old_slack, slack);
      assign_set_colslack(assign, x, slack);
    }
}

void assign_copy_colsum_to_extra(assign_mat *assign, assign_mat *assign_from) {
  size_t x;
  
  for (x = 0; x < assign->width; x++) {
    assign->data[_assign_extra(assign, x)] = assign_colsum(assign_from, x);
  }
}
void assign_copy_rowsum_to_extra(assign_mat *assign, assign_mat *assign_from) {
  size_t y;
  
  for (y = 0; y < assign->height; y++) {
    assign->data[_assign_extra(assign, y)] = assign_rowsum(assign_from, y);
  }
}

double assign_normalize_cols(assign_mat *assign, assign_mat *assign_prev, double max_slack) {
  size_t x, y;

  double sumsum = 0;

  for (x = 0; x < assign->width; x++) {
    double sum = assign_colsum(assign_prev, x);
    sumsum += sum;
    double inv_sum = 1.0 / sum;
    // printf("assign_colsum(assign_prev, x): %f\n", assign_colsum(assign_prev, x));
    // printf("inv_sum: %f\n", inv_sum);
    for (y = 0; y < assign->height; y++) {
      double val = assign_get(assign_prev, x, y) * inv_sum;
      assign_set(assign, x, y, val);
    }

    double val = assign_colslack(assign_prev, x);
    val *= inv_sum;
    // val = min(val, max_slack);
    assign_set_colslack(assign, x, val);
  }

  return sumsum;
}

double assign_normalize_rows(assign_mat *assign, assign_mat *assign_prev, double max_slack) {
  size_t x, y;

  double sumsum = 0;
  
  for (y = 0; y < assign->height; y++) {
    double sum = assign_rowsum(assign, y);
    sumsum += sum;
    double inv_sum = 1.0 / sum;
    // printf("assign_rowsum(assign, y): %f\n", assign_rowsum(assign, y));
    // printf("inv_sum: %f\n", inv_sum);
    for (x = 0; x < assign->width; x++) {
      double val = assign_get(assign_prev, x, y) * inv_sum;
      assign_set(assign, x, y, val);
    }

    double val = assign_rowslack(assign, y);
    val *= inv_sum;
    // val = min(val, max_slack);
    assign_set_rowslack(assign, y, val);
  }

  return sumsum;
}

double assign_norm_diff(assign_mat *assign, assign_mat *assign_prev) {
  size_t x, y;
  double assign_norm = 0;

  for (x = 0; x < assign->width; x++) {
    double val = assign_colslack(assign, x);
    double prev = assign_colslack(assign_prev, x);
    assign_norm += pow(val - prev, 2.0);
  }
  for (y = 0; y < assign->height; y++) {
    for (x = 0; x < assign->width; x++) {
      double val = assign_get(assign, x, y);
      double prev = assign_get(assign_prev, x, y);
      assign_norm += pow(val - prev, 2.0);
    }

    double val = assign_rowslack(assign, y);
    double prev = assign_rowslack(assign_prev, y);
    assign_norm += pow(val - prev, 2.0);
  }

  return assign_norm;
}

// softassign
assign_mat *assign_normalize(assign_mat *assign2, assign_mat *assign1, double small, double max_slack) {
  // Sinkhorn's method?
  double assign_norm = 0, last_assign_norm = 0;
  assign_mat *assign = assign2;
  assign_mat *assign_prev = assign1;
  int iters = 0;
  do {
    last_assign_norm = assign_norm;
    assign_norm = 0;

    if (SPDEBUG_ASSIGN > 1) { printf("assign.0:\n"); assign_print(assign_prev); }

    // save a copy of what rowsum is before normalizing
    // assign_copy_rowsum_to_extra(assign, assign_prev);
    assign_normalize_cols(assign, assign_prev, max_slack);
    if (SPDEBUG_ASSIGN > 1) { printf("assign.2:\n"); assign_print(assign); }
    // assign_rescale_rowslack(assign, assign_prev, max_slack);
    // printf("assign.5:\n"); assign_print(assign);

    // first normalize copied from prev into assign, so we have to
    // read rowsums from assign and write to assign here
    // assign_copy_colsum_to_extra(assign, assign);
    assign_normalize_rows(assign, assign, max_slack);
    if (SPDEBUG_ASSIGN > 1) { printf("assign.8:\n"); assign_print(assign); }
    // assign_rescale_colslack(assign, assign, max_slack);
    // printf("assign.9:\n"); assign_print(assign);

    assign_norm = assign_norm_diff(assign, assign_prev);
    
    // if (SPDEBUG_ASSIGN) {
    //   printf("assign_norm: %f  sum: %f  sqsum: %f  slack_sum: %f\n", assign_norm, assign->assign_sum, assign_sqsum(assign), assign->slack_sum);
    //   printf("assign:\n"); assign_print(assign);
    // }

    if (!isfinite(assign_norm)) {printf("Bad assign_norm: %f\n", assign_norm); abort();}

    // swap assign and assign_prev
    assign = assign_prev;
    assign_prev = assign == assign1? assign2 : assign1;
    // abort();
    iters++;
  } while (iters < 50 || (assign_norm > small && abs(last_assign_norm - assign_norm) > 0.0000001)); // what is small here?

  // swap back
  assign = assign_prev;
  assign_prev = assign == assign1? assign2 : assign1;

  if (SPDEBUG_ASSIGN) {
    printf("assign_norm: %f  sum: %f  sqsum: %f  slack_sum: %f\n", assign_norm, assign->assign_sum, assign_sqsum(assign), assign->slack_sum);
    // printf("assign_prev:\n"); assign_print(assign);
    printf("assign done after %d iterations:\n", iters); assign_print(assign);
  }

  return assign_prev;
}

// new normalization method: compute sum and then normalize everything by f(rowsum, colsum) where f = max or f = average, etc.
// average gets stuck when for example, rowsums > 1 and colsums < 1 and (rowsum+colsum)/2 is near 0
// max doesn't work, needs something like greatest geometric distance from 1 instead
assign_mat *assign_normalize2(assign_mat *assign2, assign_mat *assign1, double small, double max_slack) {
  double assign_norm = 0, last_assign_norm = 0;
  assign_mat *assign = assign2;
  assign_mat *assign_prev = assign1;
  size_t x, y, w = assign->width, h = assign->height;
  do {
    last_assign_norm = assign_norm;
    assign_norm = 0;

    for (x = 0; x < w; x++) {
      assign_set_extra(assign_prev, x, assign_colslack(assign_prev, x));
    }
    for (y = 0; y < h; y++) {
      assign_set_extra(assign_prev, w + y, assign_rowslack(assign_prev, y));
    }
    for (y = 0; y < h; y++) {
      for (x = 0; x < w; x++) {
        double val = assign_get(assign_prev, x, y);
        assign_prev->data[_assign_extra(assign_prev, x)] += val;
        assign_prev->data[_assign_extra(assign_prev, w+y)] += val;
      }
    }

    for (y = 0; y < h; y++) {
      for (x = 0; x < w; x++) {
        double val = assign_get(assign_prev, x, y);
        double rowsum = assign_extra(assign_prev, x);
        double colsum = assign_extra(assign_prev, w+y);
        // val /= (rowsum + colsum)/2;
        // val /= max(rowsum, colsum); // doesn't work when sum < 1
        if (max(rowsum, 1/rowsum) > max(colsum, 1/colsum)) {
          val /= rowsum;
        } else {
          val /= colsum;
        }
        assign_set(assign, x, y, val);
      }
    }

    assign_norm = assign_norm_diff(assign, assign_prev);
    
    if (SPDEBUG_ASSIGN) {
      printf("2 assign_norm: %f  sum: %f  sqsum: %f  slack_sum: %f\n", assign_norm, assign->assign_sum, assign_sqsum(assign), assign->slack_sum);
      printf("2 assign:\n"); assign_print(assign);
    }

    // swap assign and assign_prev
    assign = assign_prev;
    assign_prev = assign == assign1? assign2 : assign1;
  } while (assign_norm > small && abs(last_assign_norm - assign_norm) > 0.001); // what is small here?

  if (SPDEBUG_ASSIGN) {
    // printf("assign_prev:\n"); assign_print(assign);
    printf("2 assign:\n"); assign_print(assign_prev);
  }

  return assign_prev;
}

// TODO: better name?
// Convert assign to a pure 0/1 matrix by selecting as 1 all items
// that are the max in both their row and column
void assign_finish(assign_mat *assign) {
  size_t x, y;

  // fill extra with 0
  for (x = 0; x < assign->width; x++) {
    assign_set_extra(assign, x, 0);
  }

  // set extra to max in each col/row
  for (y = 0; y < assign->height; y++) {
    double rowmax = 0;
    for (x = 0; x < assign->width; x++) {
      double val = assign_get(assign, x, y);
      rowmax = max(rowmax, val);
      assign_set_extra(assign, x, max(val, assign_extra(assign, x)));
    }
    assign_set_extra(assign, assign->width+y, rowmax);
  }

  for (y = 0; y < assign->height; y++) {
    double rowmax = assign_extra(assign, assign->width+y);

    for (x = 0; x < assign->width; x++) {
      double colmax = assign_extra(assign, x);
      double val = assign_get(assign, x, y);
      if (val == rowmax && val == colmax) {
        printf("1");
        val = 1;
      } else {
        if (val == rowmax) {
          printf("-");
        } else if (val == colmax) {
          printf("!");
        } else {
          printf(".");
        }
        val = 0;
      }
      assign_set(assign, x, y, val);
    }
    printf("\n");
  }
}

cv::Vec4d v3_to_v4(cv::Vec3d v, float w) {
  return cv::Vec4d(v[0], v[1], v[2], w);
}
cv::Vec3d v4_to_v3(cv::Vec4d v) {
  return cv::Vec3d(v[0], v[1], v[2]);
}

void print_vec(const char* name, cv::Vec4d v) {
  printf("%s: [%g, %g, %g, %g]\n", name, v[0], v[1], v[2], v[3]);
}
void print_vec(const char* name, cv::Vec3d v) {
  printf("%s: [%g, %g, %g]\n", name, v[0], v[1], v[2]);
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

  // most of these will be set to something else in softposit_init
  data->beta_final = 0.85;
  data->beta_update = 1.05;
  data->small = 0.00001;
  data->focal_length = 1.0; // ?
  //data->alpha = 0; //0.0001; // TODO: This is often computed based on noise in the image.
  data->alpha = 5.0 * pow (9.21/715.0, 2.0); // Approximately 5 pixel error, with 0.99 probability?
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
  double beta,
  double imgsize,
  double objsize
);
double softposit_squared_dists(
  softposit_data *data,
  const std::vector<cv::Point2f> &image_points,
  double slack,
  double beta
);

void softposit_init(
  softposit_data *data,
  const std::vector<cv::Point2f> &image_points,
  double slack,
  double *imgsize,
  double *objsize
) {
  size_t i, o, j, k, jj;
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
  *imgsize = max(xmax - xmin, ymax - ymin)/2;

  cv::Vec4d best_pose1;
  cv::Vec4d best_pose2;
  double best_sqsum = 2000000000000.0;

  // initialize correction
  for (k = 0; k < data->num_object_points; k++) {
    data->correction[k] = 1;
  }

  for (o = 0, k = 0; o < data->objects.size(); o++) {
    obj = data->objects[o];

    *objsize = 0;

    for (i = 0; i < obj->points.size(); i++, k++) {
      if (SPDEBUG_INIT) printf("%ld ", k);
      if (SPDEBUG_INIT) print_vec("objpoint", obj->points[i]);
      double len = cv::norm(obj->points[i]);
      *objsize = max(len, *objsize);
    }

    double dist = *objsize / *imgsize;
    if (SPDEBUG_INIT) {
      printf("x: min: %f  max: %f\n", xmin, xmax);
      printf("y: min: %f  max: %f\n", ymin, ymax);
      printf("imgsize: %f\n", *imgsize);
      printf("obj %ld objsize: %f\n", o, *objsize);
      printf("dist: %f\n", dist);
      printf("data->alpha: %f\n", data->alpha);
    }


    // avg distsq of every image point to its closest neighbor
    // TODO: Better algo than this O(n^2)?
    double avgimgdistsq = 0;
    for (j = 0; j < image_points.size(); j++) {
      x = image_points[j].x;
      y = image_points[j].y;
      double best_idistsq = INFINITY;
      for (jj = 0; jj < image_points.size(); jj++) {
        if (j == jj) continue;

        double idistsq = pow(x - image_points[jj].x, 2.0) + pow(y - image_points[jj].y, 2.0);
        if (idistsq < best_idistsq) {
          best_idistsq = idistsq;
        }
      }
      avgimgdistsq += best_idistsq;
    }
    avgimgdistsq /= (double)image_points.size();
    //data->beta_final = -log(slack) / avgimgdistsq * 2;

    // TODO: Calculate pose vectors from initial rot/trans

    cv::Vec4d pose1, pose2;

    // try a bunch of different random poses, see which one matches best
    for (size_t p = 0; p < 100; p++) {
      // there's no guarantee that these will be orthogonal
      // a better randomization that makes orthogonal vectors might help
      do {
        // Q1 = s(R1, Tx)
        // Q2 = s(R2, Ty)
        // s = f/Tz
        randu(pose1, cv::Scalar(-1.0), cv::Scalar(1.0));
        randu(pose2, cv::Scalar(-1.0), cv::Scalar(1.0));

        // try constraining the pose to be mostly upright?
#if 0
        pose1[1] /= 1000.0;
        pose2[0] /= 100.0;
        pose2[1] = abs(pose2[1]);
        pose2[2] /= 100.0;

        //hack: set to known values for test
        pose1[0] = 0.0;
        pose1[1] = 1.0;
        pose1[2] = 0.0;
        pose2[0] = 0.0;
        pose2[1] = 1.0;
        pose2[2] = 0.0;
#endif

        pose1[3] = 0;
        pose2[3] = 0;
        pose1 /= cv::norm(pose1) * dist;
        pose2 /= cv::norm(pose2) * dist;
        pose1[3] = (xmax + xmin) / 2.0; // s*Tx
        pose2[3] = (ymax + ymin) / 2.0; // s*Ty
        if (!isfinite(pose1[0])) {
          printf("Bad: %f %f %f\n", *imgsize, *objsize, dist);
          print_vec("pose1", pose1);
          print_vec("pose2", pose2);
        }
      } while (!isfinite(pose1[0]));

      // printf("norms %d %d %d %d\n", isinf(norm1), isnan(norm1), isinf(norm2), isnan(norm2));
      obj->pose1 = pose1;
      obj->pose2 = pose2;

      if (SPDEBUG_INIT) print_vec("init pose1", obj->pose1);
      if (SPDEBUG_INIT) print_vec("init pose2", obj->pose2);

      // initialize correction
      for (k = 0; k < data->num_object_points; k++) {
        data->correction[k] = 1;
      }

      // Clear the assignment matrices
      assign_set_all_slack(&data->assign1, slack);
      assign_set_all_slack(&data->assign2, slack);

      data->beta_init = 0.445310; // 0.001; // -log(slack) / pow(*imgsize, 2.0); //avgdistsq;

      double avgdistsq = softposit_squared_dists(data, image_points, slack, data->beta_init);
      // initial beta should make the exp(-beta*distsq) for average distsq the same as slack
      // beta_final should be calculated similar, but we will use the expected distsq at the end
      // of the algorithm. A good value for expected distsq is probably something related to the
      // avg/smallest distsq of every image point to its closest neighbor?
      if (SPDEBUG_INIT) {
        printf("avgdistsq: %f   img: %f\n", avgdistsq, avgimgdistsq);
        printf("beta_init: %f   final: %f\n", data->beta_init, data->beta_final);
        printf("beta_update: %f \n", data->beta_update);
      }
      
      // deterministic annealing loop
      double beta = data->beta_init; //0.0004; // anealing amount? TODO: Better name
      double beta_final = data->beta_final;
      while (beta < beta_final) {
        if (SPDEBUG_LOOP) printf("\n\nsoftposit loop start beta: %f\n", beta);
        softposit_one(data, image_points, slack, beta, *imgsize, *objsize);
        beta *= data->beta_update;
        // break;
      }

      // run a few iterations of the loop just for fun
      // softposit_one(data, image_points, slack, 0.0004);
      // softposit_one(data, image_points, slack, 0.004);
      // softposit_one(data, image_points, slack, 0.04);
      // assign_mat *assign = softposit_one_setup(data, image_points, slack, 0.4);
      // printf("random assign:  sum: %f  sqsum: %f  slacksum: %f\n", assign->assign_sum, assign_sqsum(assign), assign->slack_sum);

      // using sqsum is an attempt to approximate how specific the assign matrix is.
      // values should be between 0 and 1 (TODO: they aren't always, why?), so squaring
      // the values should make values near 1 stay the same, while values closer to 0
      // go way closer to 0. A sqsum value near the number of image points is probably
      // the best goal. 
      avgdistsq = softposit_squared_dists(data, image_points, slack, 1);
      if (avgdistsq < best_sqsum) {
        best_sqsum = avgdistsq;
        best_pose1 = pose1;
        best_pose2 = pose2;
      }
    }

    // printf("best_sqsum: %f\n", best_sqsum);

    obj->pose1 = best_pose1;
    obj->pose2 = best_pose2;
  }
}

// returns average distsq
double softposit_squared_dists(
  softposit_data *data,
  const std::vector<cv::Point2f> &image_points,
  double slack,
  double beta
) {
  size_t i, o, j, k;
  Object *obj;

  double distsqsum = 0;

  for (o = 0, k = 0; o < data->objects.size(); o++) {
    obj = data->objects[o];

    if (SPDEBUG_DISTSQ) {
      print_vec("pose1", obj->pose1);
      print_vec("pose2", obj->pose2);
    }
    for (i = 0; i < obj->points.size(); i++, k++) {
      double dot1 = obj->pose1.dot(v3_to_v4(obj->points[i], 1));
      double dot2 = obj->pose2.dot(v3_to_v4(obj->points[i], 1));
      if (SPDEBUG_DISTSQ) {
        printf("%ld %ld ", i, k);
        print_vec("point", obj->points[i]);
        printf("correction %f dots: %f %f\n", data->correction[k], dot1, dot2);
      }
      if (isnan(dot1)) {
        print_vec("pose1", obj->pose1);
        print_vec("pose2", obj->pose2);
        printf("norms %f %f\n", cv::norm(obj->pose1), cv::norm(obj->pose2));
        printf("%ld %ld ", i, k);
        print_vec("point", v3_to_v4(obj->points[i], 1));
        printf("dot: %f %f\n", dot1, dot2);
        printf("abort isnan(dot1)\n");
        abort();
      }

      if (SPDEBUG_DISTSQ_V) printf("distsq:");
      for (j = 0; j < image_points.size(); j++) {
        
        // compute squared distances
        double distsq = 
          pow(dot1 - data->correction[k] * image_points[j].x, 2.0) + 
          pow(dot2 - data->correction[k] * image_points[j].y, 2.0);
        
        distsqsum += distsq;
        
        // this is the formula from the paper, but if you use it then the
        // assign matrix never seems to try to assign any blobs to leds,
        // if you sum the squares of all the values, it is always near 0.
        double x = -beta * (distsq - data->alpha) / data->alpha;
        double val = slack * exp(x);
        if (SPDEBUG_DISTSQ_V) printf("slack %f beta: %f  distsq: %f, alpha: %f\n", slack, beta, distsq, data->alpha);

        if (SPDEBUG_DISTSQ) printf("   %f dist %f -> %f\n", x, distsq, val);
        assign_set(&data->assign1, j, k, val);
      }
      if (SPDEBUG_DISTSQ) printf("\n");
    }
  }
  if (SPDEBUG_DISTSQ) {
    printf("after distsq\n");
    assign_print(&data->assign1);
  }
  return distsqsum / (data->assign1.width * data->assign1.height);
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

  if (abs(obj->pose1[0]) + abs(obj->pose1[1]) + abs(obj->pose1[2]) + abs(obj->pose2[0]) + abs(obj->pose2[1]) + abs(obj->pose2[2]) < 0.000000001) {
    print_mat("L_inv", &L_inv);
    print_vec("pose1", obj->pose1);
    print_vec("pose2", obj->pose2);
    printf("abort pose1 pose2 = 0\n");
    // abort();
    // obj->pose1[0] = 0.1;
    // obj->pose2[1] = 0.1;
  }
}

// returns the cross product
cv::Vec3d make_orthogonal_v3(cv::Vec3d &a, cv::Vec3d &b, bool unitlen) {
  cv::Vec3d r1 = a / cv::norm(a);
  cv::Vec3d r2 = b / cv::norm(b);
  cv::Vec3d r3 = r1.cross(r2);
  cv::Vec3d r1a, r2a;
  r1a = r2.cross(r3);
  r1a /= cv::norm(r1a);
  r2a = r3.cross(r1);
  r2a /= cv::norm(r2a);

  r1 = (r1 + r1a)/2;
  r1 /= cv::norm(r1);
  r2 = (r2 + r2a)/2;
  r2 /= cv::norm(r2);

  if (unitlen) {
    a = r1;
    b = r2;
  } else {
    a = r1 * a.dot(r1);
    b = r2 * b.dot(r2);
  }

  return r3;
}

// makes the vec3 components orthogonal without changing the 4th
void make_orthogonal_v3_in_v4(cv::Vec4d &a, cv::Vec4d &b, bool unitlen) {
  double av = a[3], bv = b[3];
  cv::Vec3d a3 = v4_to_v3(a), b3 = v4_to_v3(b);
  make_orthogonal_v3(a3, b3, unitlen);
  a = v3_to_v4(a3, av);
  b = v3_to_v4(b3, bv);
}

void softposit_compute_rot_trans(
  softposit_data *data,
  Object *obj,
  double imgsize,
  double objsize
) {
  double n1 = cv::norm(v4_to_v3(obj->pose1));
  double n2 = cv::norm(v4_to_v3(obj->pose2));
  if (SPDEBUG_ROTTRANS > 2) {
    printf("n1: %f, n2: %f\n", n1, n2);
    printf("n1 * n2: %f\n", n1 * n2);
    printf("sqrt(n1 * n2): %f\n", sqrt(n1 * n2));
  }
  double s_inv = 1.0/sqrt(
    cv::norm(v4_to_v3(obj->pose1)) *
    cv::norm(v4_to_v3(obj->pose2))
  );

  if (SPDEBUG_ROTTRANS) printf("s_inv: %.1e\n", s_inv);
  if (isinf(s_inv)) {
    printf("abort isinf(s_inv)\n");
    abort();
  }

  cv::Vec3d r1 = v4_to_v3(obj->pose1);
  // r1 /= cv::norm(r1);
  // printf("r1 (%f) = ", cv::norm(r1)); print_vec("pose1 * s_inv", r1);
  cv::Vec3d r2 = v4_to_v3(obj->pose2);
  // r2 /= cv::norm(r2);
  // printf("r2 (%f) = ", cv::norm(r2)); print_vec("pose2 * s_inv", r2);
  cv::Vec3d r3 = make_orthogonal_v3(r1, r2, true);
  // r3 /= cv::norm(r3);
  // // printf("r3 (%f) = ", cv::norm(r3)); print_vec("r1 x r2", r3);

  // // make r1 and r2 orthogonal
  if (SPDEBUG_ROTTRANS > 2) printf("r1 . r2: %f\n", r1.dot(r2));
  // cv::Vec3d r1a, r2a;
  // r1a = r2.cross(r3);
  // r1a /= cv::norm(r1a);
  // r2a = r3.cross(r1);
  // r2a /= cv::norm(r2a);
  // // printf("%f ", r1.dot(r1a)); print_vec("r1a", r1a);
  // // printf("%f ", r2.dot(r2a)); print_vec("r2a", r2a);
  // r1 = (r1 + r1a)/2;
  // r1 /= cv::norm(r1);
  // r2 = (r2 + r2a)/2;
  // r2 /= cv::norm(r2);
  // // printf("r1 . r2: %f\n", r1.dot(r2));


  obj->rotation.at<double>(0, 0) = r1[0];
  obj->rotation.at<double>(0, 1) = r1[1];
  obj->rotation.at<double>(0, 2) = r1[2];
  obj->rotation.at<double>(1, 0) = r2[0];
  obj->rotation.at<double>(1, 1) = r2[1];
  obj->rotation.at<double>(1, 2) = r2[2];
  obj->rotation.at<double>(2, 0) = r3[0];
  obj->rotation.at<double>(2, 1) = r3[1];
  obj->rotation.at<double>(2, 2) = r3[2];
  if (SPDEBUG_ROTTRANS) print_mat("rot", &obj->rotation);

  // printf("softposit compute trans\n");

  obj->translation[0] = obj->pose1[3] * s_inv;
  obj->translation[1] = obj->pose2[3] * s_inv;
  obj->translation[2] = data->focal_length * s_inv; // TODO: why /2?
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
      data->correction[k] = cv::Vec3d(obj->rotation.row(2)).dot(obj->points[i]/obj->translation[2]) + 1;
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
    //assign_set_all_slack(&data->assign1, slack /*min(slack, beta * 2)*/);

    double avgdistsq = softposit_squared_dists(data, image_points, slack, beta);
    if (SPDEBUG_LOOP) printf("avgdistsq: %f\n", avgdistsq);

    // printf("assign1:\n"); assign_print(&data->assign1);

    return assign_normalize(&data->assign2, &data->assign1, data->small, 1 /*beta * 2*/);
}

void softposit_one(
  softposit_data *data,
  const std::vector<cv::Point2f> &image_points,
  double slack,
  double beta,
  double imgsize,
  double objsize
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
      if (SPDEBUG_LOOP) {
        print_vec("pose1", obj->pose1);
        print_vec("pose2", obj->pose2);
        printf("pose1 len4: %f  len3: %f\n", cv::norm(obj->pose1), cv::norm(v4_to_v3(obj->pose1)));
        printf("pose2 len4: %f  len3: %f\n", cv::norm(obj->pose2), cv::norm(v4_to_v3(obj->pose2)));
        printf("dot 4: %f   dot 3: %f\n", obj->pose1.dot(obj->pose2), v4_to_v3(obj->pose1).dot(v4_to_v3(obj->pose2)));
        make_orthogonal_v3_in_v4(obj->pose1, obj->pose2, false);
        printf("orthogonalized\n");
        print_vec("pose1", obj->pose1);
        print_vec("pose2", obj->pose2);
        printf("pose1 len4: %f  len3: %f\n", cv::norm(obj->pose1), cv::norm(v4_to_v3(obj->pose1)));
        printf("pose2 len4: %f  len3: %f\n", cv::norm(obj->pose2), cv::norm(v4_to_v3(obj->pose2)));
        printf("dot 4: %f   dot 3: %f\n", obj->pose1.dot(obj->pose2), v4_to_v3(obj->pose1).dot(v4_to_v3(obj->pose2)));
      }

      // if (z1 || z2) {
      //   some_zero = true;
      //   continue;
      // }
      

      softposit_compute_rot_trans(data, obj, imgsize, objsize);

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

  size_t num_image_points = image_points.size();
  size_t num_object_points = data->num_object_points;

  size_t size = num_object_points > num_image_points? num_object_points : num_image_points;
  double slack = 1.0/(size + 1.0); // better name?
  if (SPDEBUG_ROTTRANS) printf("slack: %f\n", slack);

  double imgsize, objsize;

  printf("b4 data->alpha: %f\n", data->alpha);
  softposit_init(data, image_points, slack, &imgsize, &objsize);

#if 0
  // printf("softposit init\n");
  // initialize slack elements of assign
  assign_set_all_slack(&data->assign1, slack);
  assign_set_all_slack(&data->assign2, slack);

  // if (SPDEBUG_ROTTRANS) {
  //   printf("assign1:\n"); assign_print(&data->assign1);
  //   printf("assign2:\n"); assign_print(&data->assign2);
  // }

  // this can be larger with a guessed initial pose
  double beta = data->beta_init; //0.0004; // anealing amount? TODO: Better name
  double beta_final = data->beta_final;
  double beta_update = data->beta_update;

  char *line = NULL;
  size_t n = 0;

  // deterministic annealing loop
  while (beta < beta_final) {

    if (SPDEBUG_LOOP) printf("\n\nsoftposit loop start beta: %f\n", beta);
    if (SPDEBUG_KBWAIT) if (!(n > 2 && line[0] == 'g' && line[1] == 'o')) {
      printf("waiting:");
      getline(&line, &n, stdin);
    }

    softposit_one(data, image_points, slack, beta, imgsize, objsize);

    beta *= beta_update;
    // break;
    // abort();
  }

  free(line);
#endif

  if (SPDEBUG_DONE) {
    printf("softposit done\n");
    printf("final assign:\n"); assign_print(&data->assign1);
    assign_finish(&data->assign1);
    print_mat("rot", &data->objects[0]->rotation);
    print_vec("trans", data->objects[0]->translation);
  }
    // abort();
}

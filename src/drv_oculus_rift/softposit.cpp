// Copyright 2019, Google.

#include "softposit.h"

using namespace std;


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


typedef struct {
  size_t width, height;
  size_t data_len;
  double* data;
} assign_mat;

void assign_resize(assign_mat *mat, size_t width, size_t height);

assign_mat assign_mat_new(size_t width, size_t height) {
  assign_mat mat;
  mat.width = 0;
  mat.height = 0;
  mat.data_len = 0;
  mat.data = NULL;
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
  for (size_t y = 0; y < mat->height; y++) {
    for (size_t x = 0; x < mat->width; x++) {
      printf("%f ", assign_get(mat, x, y));
    }
    printf("%f %f\n", assign_rowslack(mat, y), assign_rowsum(mat, y));
  }
  for (size_t x = 0; x < mat->width; x++) {
    printf("%f ", assign_colslack(mat, x));
  }
  printf("\n");
  for (size_t x = 0; x < mat->width; x++) {
    printf("%f ", assign_colsum(mat, x));
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
    for (y = 0; y < assign->height; y++) {
      assign->data[_assign_extra(assign, y)] = assign_rowsum(assign_prev, y);
    }

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
      double old_sum = assign->data[_assign_extra(assign, y)];
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

    printf("assign_norm: %f\n", assign_norm);
    printf("assign:\n"); assign_print(assign);

    // swap assign and assign_prev
    assign = assign_prev;
    assign_prev = assign == assign1? assign2 : assign1;
  } while (assign_norm > small && abs(last_assign_norm - assign_norm) > 0.001); // what is small here?

  printf("assign:\n"); assign_print(assign_prev);

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
      printf("%f ", v[i]);
  }
    printf("]\n");
}
void print_mat(const char* name, cv::Mat* m) {
  printf("%s: \n", name);
  for (int i = 0; i < m->rows; i++) {
    printf("    ");
    for (int j = 0; j < m->cols; j++) {
      printf("%f ", m->at<double>(i, j));
    }
    printf("\n");
  }
}

void softposit(
  std::vector<Object*> objects, 
  std::vector<cv::Point2f> image_points
  ) {
  // this can be larger with a guessed initial pose
  double beta = 0.0004; // anealing amount? TODO: Better name
  double beta_final = 0.5; //0.5;
  double beta_update = 1.05; // must be > 1, multiplied to beta until beta_final
  double small = 0.01; // TODO: This is a guess, What should this be?
  double focal_length = 1.0; // TODO: What should this be?

  double alpha = 0.01; // TODO: This needs to be computed somehow!

  size_t num_image_points = image_points.size();
  size_t num_objects = objects.size();
  size_t num_object_points = 0;

  size_t i, o, j, k;



  // printf("softposit start\n");

  for (o = 0, k = 0; o < num_objects; o++) {
    num_object_points += objects[o]->points.size();
    // objects[o]->pose1 = cv::Scalar(1);
    // objects[o]->pose2 = cv::Scalar(1);
    randu(objects[o]->pose1, cv::Scalar(-10.0), cv::Scalar(10.0));
    randu(objects[o]->pose2, cv::Scalar(-10.0), cv::Scalar(10.0));

    for (i = 0; i < objects[o]->points.size(); i++, k++) {
      printf("%ld ", k);
      print_vec("objpoint", v3_to_v4(objects[o]->points[i], 1));
    }
  }
  for (j = 0; j < num_image_points; j++) {
    printf("%ld imgpoint: %f %f \n", j, image_points[j].x, image_points[j].y);
  }

  // printf("num_objects: %i\n", num_objects);
  // printf("num_object_points: %i\n", num_object_points);

  std::vector<double> correction(num_object_points); // correction terms (w)
  assign_mat assign1 = assign_mat_new(num_image_points, num_object_points);
  assign_mat assign2 = assign_mat_new(num_image_points, num_object_points);
  // cv::Mat assign1 = cv::Mat(num_image_points+1, num_object_points+1, CV_64F, cv::Scalar(0)); // (m i)
  // cv::Mat assign2 = cv::Mat(num_image_points+1, num_object_points+1, CV_64F, cv::Scalar(0)); // (m i-1) -- alternates with assign1
  // std::vector<double> images_sum(num_image_points+1); // sum of each row in assign
  // std::vector<double> objects_sum(num_object_points+1); // sum of each column in assign

  size_t size = num_object_points > num_image_points? num_object_points : num_image_points;
  double slack = 1.0/(2*size + 1.0); // better name?
  printf("slack: %f\n", slack);

  // printf("softposit init\n");
  // initialize slack elements of assign
  assign_set_all_slack(&assign1, slack);

  // initialize correction
  for (k = 0; k < num_object_points; k++) {
    correction[k] = 1;
  }

  // deterministic annealing loop
  while (beta < beta_final) {

    printf("\n\nsoftposit loop start beta: %f\n", beta);

    // is this a good idea?
    assign_set_all_slack(&assign1, min(slack, beta * 2));

    for (o = 0, k = 0; o < num_objects; o++) {
      // print_vec("pose1", objects[o]->pose1);
      // print_vec("pose2", objects[o]->pose2);
      for (i = 0; i < objects[o]->points.size(); i++, k++) {
        // printf("%ld %ld ", i, k);
        // print_vec("point", v3_to_v4(objects[o]->points[i], 1));
        double dot1 = objects[o]->pose1.dot(v3_to_v4(objects[o]->points[i], 1));
        double dot2 = objects[o]->pose2.dot(v3_to_v4(objects[o]->points[i], 1));
        // printf("dot1: %f\n", dot1);
        if (isnan(dot1)) abort();

        for (j = 0; j < num_image_points; j++) {
          
          // compute squared distances
          double distsq = 
            pow(dot1 - correction[k] * image_points[j].x, 2.0) + 
            pow(dot2 - correction[k] * image_points[j].y, 2.0);
          
          distsq += (rand() % 100) / 1000.0;
          // printf("distsq: %f\n", distsq);
          
          // initialize assign
          double val = slack * exp(-beta * (distsq - alpha)); // what is alpha?
          assign_set(&assign1, j, k, val);
        }
      }
    }

    printf("assign1:\n"); assign_print(&assign1);


    printf("softposit softassign\n");

    assign_mat *assign = assign_normalize(&assign2, &assign1, small, beta * 2);

    // print_vector("images_sum", images_sum);
    // print_vector("objects_sum", objects_sum);

    // printf("softposit compute L\n");

    // compute 4x4 matrix L TODO: Better name?
    cv::Mat L(4, 4, CV_64F, cv::Scalar(0));
    for (o = 0, k = 0; o < num_objects; o++) {
      for (i = 0; i < objects[o]->points.size(); i++, k++) {
        // printf("L 1...");
        cv::Mat a = cv::Mat(v3_to_v4(objects[o]->points[i], 1));
        // print_mat("L a", &a);
        cv::Mat tmp = cv::Mat(a * a.t());
        // print_mat("L tmp", &tmp);
        // printf("objects_sum[k]: %f\n", objects_sum[k]);
        L += assign_rowsum(assign, k) * tmp;
        // printf("L 4...");
      }
    }

    // L /= 10000.0;
    print_mat("L", &L);

    cv::Mat L_inv = L.inv();
    print_mat("L_inv", &L_inv);

    // compute new Q1 and Q2
    cv::Vec4d tmp;
    for (o = 0, k = 0; o < num_objects; o++) {
      objects[o]->pose1 = cv::Scalar(0);
      objects[o]->pose2 = cv::Scalar(0);

      // printf("softposit compute pose1 pose2\n");

      for (i = 0; i < objects[o]->points.size(); i++, k++) {
        for (j = 0; j < num_image_points; j++) {
          tmp = assign_get(assign, j, k) * correction[k] * v3_to_v4(objects[o]->points[i], 1);
          print_vec("tmp", tmp);
          cv::Vec4d a = objects[o]->pose1 + tmp * image_points[j].x;
          print_vec("a", a);
          objects[o]->pose1 = a;
          print_vec("pose1", objects[o]->pose1);
          objects[o]->pose2 = cv::Vec4d(objects[o]->pose2 + tmp * image_points[j].y);
        }
      }
      
      objects[o]->pose1 = cv::Mat(L_inv * objects[o]->pose1);
      objects[o]->pose2 = cv::Mat(L_inv * objects[o]->pose2);

      print_vec("pose1", objects[o]->pose1);
      print_vec("pose2", objects[o]->pose2);

      // printf("softposit compute s_inv\n");

      double s_inv = 1.0/sqrt(
        sqrt(pow(objects[o]->pose1[0], 2.0) + pow(objects[o]->pose1[1], 2.0) + pow(objects[o]->pose1[2], 2.0)) *
        sqrt(pow(objects[o]->pose2[0], 2.0) + pow(objects[o]->pose2[1], 2.0) + pow(objects[o]->pose2[2], 2.0))
      );

      printf("s_inv: %f\n", s_inv);

      // printf("softposit compute rot\n");

      cv::Vec4d rtmp = objects[o]->pose1 * s_inv;
      // print_vec("pose1 * s_inv", rtmp);
      objects[o]->rotation.at<double>(0, 0) = rtmp[0];
      objects[o]->rotation.at<double>(0, 1) = rtmp[1];
      objects[o]->rotation.at<double>(0, 2) = rtmp[2];
      rtmp = objects[o]->pose2 * s_inv;
      // print_vec("pose2 * s_inv", rtmp);
      objects[o]->rotation.at<double>(1, 0) = rtmp[0];
      objects[o]->rotation.at<double>(1, 1) = rtmp[1];
      objects[o]->rotation.at<double>(1, 2) = rtmp[2];
      cv::Vec3d r3tmp = cv::Vec3d(objects[o]->rotation.row(0)).cross(cv::Vec3d(objects[o]->rotation.row(1)));
      // print_vec("r1 x r2", r3tmp);
      objects[o]->rotation.at<double>(2, 0) = r3tmp[0];
      objects[o]->rotation.at<double>(2, 1) = r3tmp[1];
      objects[o]->rotation.at<double>(2, 2) = r3tmp[2]; // R3
      print_mat("rot", &objects[o]->rotation);

      // printf("softposit compute trans\n");

      objects[o]->translation[0] = objects[o]->pose1[3] * s_inv; // T1
      objects[o]->translation[1] = objects[o]->pose2[3] * s_inv; // T2
      objects[o]->translation[2] = focal_length * s_inv; // T3
      print_vec("trans", objects[o]->translation);

    }

    // printf("softposit compute correction\n");

    for (o = 0, k = 0; o < num_objects; o++) {
      for (i = 0; i < objects[o]->points.size(); i++, k++) {
        correction[k] = cv::Vec3d(objects[o]->rotation.row(2)).dot(objects[o]->points[i])/objects[o]->translation[2] + 1;
      }
    }
    print_vector("correction", correction);

    beta *= beta_update;
    // break;
  }

  printf("softposit done\n");
  printf("assign1:\n"); assign_print(&assign1);

  assign_mat_free(&assign1);
  assign_mat_free(&assign2);
}

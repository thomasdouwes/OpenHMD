#include "softposit.h"
#include <getopt.h>

using namespace std;


cv::Vec4d mv3_to_v4(cv::Vec3d v, float w) {
  return cv::Vec4d(v[0], v[1], v[2], w);
}
cv::Vec3d mv4_to_v3(cv::Vec4d v) {
  return cv::Vec3d(v[0], v[1], v[2]);
}


void mprint_vec(const char* name, cv::Vec4d v) {
  printf("%s: [%.2f, %.2f, %.2f, %.2f]\n", name, v[0], v[1], v[2], v[3]);
}
void mprint_vec(const char* name, cv::Vec3d v) {
  printf("%s: [%.2f, %.2f, %.2f]\n", name, v[0], v[1], v[2]);
}
void mprint_vec(const char* name, cv::Vec2d v) {
  printf("%s: [%.2f, %.2f]\n", name, v[0], v[1]);
}
void mprint_mat(const char* name, cv::Mat* m) {
  printf("%s: \n", name);
  for (int i = 0; i < m->rows; i++) {
    printf("    ");
    for (int j = 0; j < m->cols; j++) {
      printf("% .2f ", m->at<double>(i, j));
    }
    printf("\n");
  }
}

cv::Mat translate(double x, double y, double z) {
  double transd[16] = {
    1,  0,  0,  0,
    0,  1,  0,  0,
    0,  0,  1,  0,
    x,  y,  z,  1,
  };
  return cv::Mat(4, 4, CV_64F, transd).clone();
}

cv::Mat scaling(double x, double y, double z) {
  double scaled[16] = {
    x,  0,  0,  0,
    0,  y,  0,  0,
    0,  0,  z,  0,
    0,  0,  0,  1,
  };
  return cv::Mat(4, 4, CV_64F, scaled).clone();
}

// cv::Mat rotation(double x, double y, double z) {
//   double scaled[16] = {
//     x,  0,  0,  0,
//     0,  y,  0,  0,
//     0,  0,  z,  0,
//     0,  0,  0,  1,
//   };
//   return cv::Mat(4, 4, CV_64F, scaled);
// }

cv::Mat projection(double n, double f) {
  double projd[16] = {
    1,  0,  0,          0,
    0,  1,  0,          0,
    0,  0,  (f+n)/(f-n),   1,
    0,  0,  -2*f*n/(n-f), 0,
  };
  return cv::Mat(4, 4, CV_64F, projd).clone();
}

cv::Vec4d apply(cv::Vec4d point, cv::Mat mat) {
  cv::Vec4d app = cv::Mat(cv::Mat(point).t() * mat);
  // mprint_vec("applied", app);
  return app;
}
cv::Vec3d apply(cv::Vec3d point, cv::Mat mat) {
  return mv4_to_v3(apply(mv3_to_v4(point, 1), mat));
}
// outputs point with x and y both >-0.5 and <0.5
// points outside that range or invalid (inv/nan) should be
// rejected as outside the camrea frame
cv::Vec2d project(cv::Vec3d point, cv::Mat mat) {
    cv::Vec4d p = cv::Mat(mat * mv3_to_v4(point, 1));
    // cv::Vec4d p = cv::Mat(cv::Mat(v3_to_v4(point, 1)).t() * mat);
    // mprint_vec("p", p);
    if (p[3] < 0) {
      return {INFINITY, INFINITY};
    }
    return {
      (p[0]) / (p[2]),
      (p[1]) / (p[2]),
    };
}
bool is_valid_2d(cv::Vec2d point) {
  return point[0] > -0.5 && point[0] < 0.5 && point[1] > -0.5 && point[1] < 0.5;
}

cv::Vec2d to_screen_space(cv::Vec2d point, double width, double height) {
  return {
    (point[0] + 0.0) * width,
    (point[1] + 0.0) * height,
  };
}

int main(int argc, char** argv)
{
  // printf("\n\n\nSoftposit test\n");

  double x = 10, y = 5, z = 40;
  double s = 1;
  double alpha = 1, beta = 0.1, beta_update = 1.1, beta_final = 1;

  int opt;
  while ((opt = getopt(argc, argv, "x:y:z:s:a:b:u:f:")) != -1) {
    switch (opt) {
      case 'x':
        x = atof(optarg);
        break;
      case 'y':
        y = atof(optarg);
        break;
      case 'z':
        z = atof(optarg);
        break;
      case 's':
        s = atof(optarg);
        break;
      case 'a':
        alpha = atof(optarg);
        break;
      case 'b':
        beta = atof(optarg);
        break;
      case 'u':
        beta_update = atof(optarg);
        break;
      case 'f':
        beta_final = atof(optarg);
        break;

      default:
        return EXIT_FAILURE;
    }
  }


  std::vector<cv::Vec3d> objpts = {
    {4, 6, 6},
    {3, 5, 5},
    {3, 1, 3},
    {10, 10, 0},
    {6, 8, -6},

    {-4, 3, 6},
    {-8, 6, 1},
    {-4, 3, -2},
    {-1, 2, -7},

    {7, -3,  6},
    {0, -14, 0},
    {4, -8, -2},
    {0, -2, -9},

    {-6, -5, 7},
    {-8, -8, -1},
    {-6, -5, -3},
    {-2, -4, -6},
  };

  cv::Vec3d camera_vec = {0, 0, 1};

  cv::Mat trans = translate(x, y, z);
  cv::Mat scale = scaling(s, s, s);

  cv::Mat proj = projection(1, 200000);

  // mprint_mat("trans", &trans);

  cv::Mat mat = proj;

  std::vector<cv::Point2f> imgpts(0);

  double focal_length = 10;

  for (size_t i = 0; i < objpts.size(); i++) {
    objpts[i] = apply(objpts[i], scale);

    if (camera_vec.dot(objpts[i]) > 0) continue; // only vectors pointing towards camera

    cv::Vec2d p = project(apply(objpts[i], trans), mat);
    // mprint_vec("2d", p);
    if (is_valid_2d(p)) {
      p = to_screen_space(p, focal_length, focal_length);
      imgpts.push_back({(float)p[0], (float)p[1]});
    }
  }

  // imgpts[objpts.size() + 0] = {0.01, 0.01};

  // double l = 0.00001;
  // printf("%f log:%f sqrtlog:%f\n", l, log(l), sqrt(-log(l)));

	Object *obj = softposit_new_object(objpts);
				 
	softposit_data *sp = softposit_new();
	softposit_add_object(sp, obj);

  sp->beta_init = beta; //0.1;
  sp->beta_update = beta_update; // 1.1;
  sp->beta_final = beta_final; //1;
  sp->focal_length = focal_length;
  sp->alpha = alpha; //0; //1/focal_length;

	softposit(sp, imgpts);


  return 0;
}

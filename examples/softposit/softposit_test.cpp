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
  //mprint_vec("applied", app);
  return app;
}
cv::Vec3d apply(cv::Vec3d point, cv::Mat mat) {
  cv::Vec3d app = mv4_to_v3(apply(mv3_to_v4(point, 1), mat));
  return app;
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
  return point[0] > -0.9 && point[0] < 0.9 && point[1] > -0.69 && point[1] < 0.66;
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

  double x = 0, y = 0, z = 1000000;
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
                {63321, -31734, 67534},
                { 77003, 1460, 65643},
                { 29006, -19167, 73205},
                { -92, -33409, 72387},
                { -29277, -19280, 73313},
                { -63317, -32109, 67604},
                { -77328, 974, 65640},
                { -54084, 34504, 68768},
                { -20154, 17780, 74139},
                { 53480, 34724, 68953},
                { 19668, 17820, 74205},
                { -78986, -12270, 28013},
                { -77680, -12236, -8572},
                { -70629, -31636, 9905},
                { -58930, -40868, 36014},
                { 59269, -40973, 36524},
                { 28475, -43555, 61577},
                { -80405, -24558, 53251},
                { -27802, -43408, 61279},
                { 70969, -31734, 10382},
                { 77642, -12844, -8764},
                { 79166, -12197, 28124},
                { 80537, -24730, 53440},
                { -28909, 43724, 57823},
                { -78119, 29616, 53275},
                { 72085, 30581, 46},
                { 66860, 37604, 29608},
                { 78145, 29213, 52728},
                { 29098, 43572, 57637},
                { -66825, 37894, 30136},
                { -72217, 30765, 462},
                { -41038, 43365, 23723},
                { 3, 43958, 33539},
                { 41065, 43154, 23442},
                { -11850, 25964, -164021},
                { -24139, 7160, -160920},
                { -32821, -11851, -153895},
                { -40456, -31212, -140108},
                { -15097, -42169, -142923},
                { 14884, -42172, -142921},
                { 40417, -31186, -139999},
                { 33046, -11639, -153922},
                { 24183, 7138, -161111},
                { 11832, 25969, -164177},
  };

  cv::Vec3d camera_vec = {0, 0, 1};

  cv::Mat trans = translate(x, y, z);
  cv::Mat scale = scaling(s, s, s);

  cv::Mat proj = projection(1, 200000);

  // mprint_mat("trans", &trans);

  cv::Mat mat = proj;

  std::vector<cv::Point2f> imgpts(0);

  double focal_length = 1;

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

#if 0
  sp->beta_init = beta; //0.1;
  sp->beta_update = beta_update; // 1.1;
  sp->beta_final = beta_final; //1;
  sp->focal_length = focal_length;
  sp->alpha = alpha; //0; //1/focal_length;
#endif

	softposit(sp, imgpts);


  return 0;
}

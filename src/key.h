using namespace std;

/*
================================
최적화된 라이다 xyz축 필터값
================================
*/
extern float minZ_filter;
extern float maxZ_filter;
extern float minY_filter;
extern float maxY_filter;
extern float minX_filter;
extern float maxX_filter;
extern float leafsize;

/*
================================
DBSCAN
================================
*/
extern int MINIMUM_POINTS;
extern float EPSILON_INPUT;
extern int totalClusterCount;

/*
================================
유클리디안
================================
*/
extern float clusterTolerance;
extern int minClusterSize;
extern int maxClusterSize;


void keyboard_input_handler(void);
int getch(void);
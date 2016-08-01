#include "mex.h"
#include <unistd.h>
#include "FHeap.h"
#include "FibNode.h"
#include "math.h"

int findMin(double *map, int rows, int cols);
int findNode(double *map, int rows, int cols);
int checkSquareCollision( double* map, int rows, int cols, int x, int y, int buffer, int unknown);
/* The computational routine */
void dijkstra(double* map, int rows, int cols, int s_i, int s_j, int unknown, double* r_map, int* path_map)
{
    
    //initialize Queue
    for(int i = 0; i < rows; i++){
        for(int j = 0; j < cols; j++){
            path_map[i*rows+j] = mxGetInf();
            r_map[i*rows+j] = mxGetInf();
            //mexPrintf("%f\n", map[i*rows+j]);
        }
    }
    FHeap<int> fh;
    fh.insert(0, s_i*rows+s_j);
    
    //FibNode<int> a;
    //fh.deleteMin(&a);
    
    //mexPrintf("Key %d, Content %d\n", a.getKey(), a.getContent());
    
    //Q[s_i*rows+s_j] = 0;
    r_map[s_i*rows+s_j] = 0;
    int loops = 0;
    while(!fh.isEmpty()){
        //fh.print();

        FibNode<int> min;
        fh.deleteMin(&min);
        int minimum = min.getContent(); //distance
        loops++;

        int i = minimum / cols;
        int j = minimum % cols;

        for(int k = -1; k <= 1; k++){
            for(int l = -1; l <= 1; l++){
                if(i+k >= 0 && i+k < rows-1 && j+l >= 0 && j+l < cols-1){
                    int feasable = 0;
                    if(unknown){
                        if(map[(i+k)*rows+(j+l)] != 3.0 && map[(i+k)*rows+(j+l)] != 1.0){
                            feasable = 1;
                        }
                    }
                    else {
                        if(map[(i+k)*rows+(j+l)] != 2.0 && map[(i+k)*rows+(j+l)] != 1.0){
                            feasable = 1;
                        }
                    }
                    /*int collision = 0;
                    for(int a = -2; a < 2; a++){
                        for(int b = -2; b < 2; b++){
                            int x_n = i+k + a;
                            int y_n = j+l + b;
                            //mexPrintf("(%d, %d), %f\n", x_n, y_n, map[i*rows+j]);
                            if(x_n >= 0 && x_n < cols-1 && y_n >= 0 && y_n < rows-1){
                                if(unknown == 0){
                                    if(map[x_n * rows +y_n] == 1.0 || map[x_n * rows +y_n] == 2.0){
                                        collision = 1;
                                    }
                                }
                                else {
                                    if(map[x_n * rows +y_n] == 1.0 || map[x_n * rows +y_n] == 3.0){
                                        collision = 1;
                                    }
                                }
                            }
                        }
                    }*/
                    if((k!=0 || l!=0) && (feasable /* && !collision*/)){
                        if(mxIsInf(r_map[(i+k)*rows+(j+l)])){
                            //mexPrintf("dist %f %d %d, %d %d\n", sqrt( (i-(i+k))*(i-(i+k)) + (j-(j+l))*(j-(j+l))), i, j, i+k, j+l);
                            double new_dist = min.getKey() + sqrt( (i-(i+k))*(i-(i+k)) + (j-(j+l))*(j-(j+l)));
                            int new_index = (i+k)*rows+(j+l);
                            fh.insert(new_dist, new_index);
                            if(new_dist < r_map[(i+k)*rows+(j+l)]){
                                path_map[(i+k)*rows+(j+l)] = minimum;// + 241;
                                r_map[(i+k)*rows+(j+l)] = new_dist;
                            }
                        }
                    }
                }
            }
        }
    }
    //Q[s_i*rows+s_j] = 255;
    //for(int i = 0; i < rows; i++){
    //    for(int j = 0; j < cols; j++){
    //        r_map[i*rows+j] = Q[i*rows+j];
    //    }
    //}
    //mexPrintf("Loops %d\n", loops);
    /*
     * double Q[rows*cols];
     *
     * //initialize Queue
     * for(int i = 0; i < rows; i++){
     * for(int j = 0; j < cols; j++){
     * Q[i*rows+j] = mxGetInf();
     * r_map[i*rows+j] = mxGetInf();
     * }
     * }
     *
     * //FHeap<int> fh;
     * //fh.insert(0, s_i*rows+s_j);
     *
     * //FibNode<int> a;
     * //fh.deleteMin(&a);
     *
     * //mexPrintf("Key %d, Content %d\n", a.getKey(), a.getContent());
     *
     * Q[s_i*rows+s_j] = 0;
     * r_map[s_i*rows+s_j] = 0;
     * int loops = 0;
     * while(findNode(Q, rows, cols)){
     * int minimum = findMin(Q, rows, cols);
     * loops++;
     * Q[minimum] = mxGetInf();
     * int i = minimum / cols;
     * int j = minimum % cols;
     * //mexPrintf("i %d, j %d\n", i, j);
     * for(int k = -1; k <= 1; k++){
     * for(int l = -1; l <= 1; l++){
     * if(i+k >= 0 && i+k < rows && j+l >= 0 && j+l < cols){
     * if(k!=0 || l!=0 ){
     * int new_dist = r_map[minimum] + 1;
     * //mexPrintf("dist %d\n", new_dist);
     * if(mxIsInf(r_map[(i+k)*rows+(j+l)])){
     * r_map[(i+k)*rows+(j+l)] = new_dist;
     * Q[(i+k)*rows+(j+l)] = r_map[(i+k)*rows+(j+l)];
     * }
     * else{
     * if(new_dist < r_map[(i+k)*rows+(j+l)]){
     * r_map[(i+k)*rows+(j+l)] = new_dist;
     * Q[(i+k)*rows+(j+l)] = r_map[(i+k)*rows+(j+l)];
     * }
     * }
     * }
     * }
     * }
     * }
     * }
     * mexPrintf("Loops %d\n", loops);*/
}

int checkSquareCollision( double* map, int rows, int cols, int x, int y, int buffer, int unknown){
    int collision = 0;
    //check square
    for(int k = -buffer/2; k < buffer/2; k++){
        for(int l = -buffer/2; l < buffer/2; l++){
            int x_n = x + k;
            int y_n = y + l;
            mexPrintf("(%d, %d), %d\n", x, y, map[y_n*rows+x_n]);
            if(x_n >= 0 && x_n < cols-1 && y_n >= 0 && y_n < rows-1){
                if(unknown == 0){
                    if(map[(x+k)*rows+(y+l)] == 1 || map[(x+k)*rows+(y+l)] == 2){
                        collision = 1;
                    }
                    else if(map[(x+k)*rows+(y+l)] == 1 || map[(x+k)*rows+(y+l)] == 3){
                        collision = 1;
                    }
                }
            }
        }
    }
    return collision;
}

int findNode(double *map, int rows, int cols){
    for(int i = 0; i < rows; i++){
        for(int j = 0; j < cols; j++){
            //int val = (int)map[i*rows+j];
            if(!mxIsInf(map[i*rows+j])){
                return 1;
            }
        }
    }
    return 0;
}

int findMin(double *map, int rows, int cols){
    int min = INT_MAX;
    int index = 0;
    for(int i = 0; i < rows; i++){
        for(int j = 0; j < cols; j++){
            if(!mxIsInf(map[i*rows+j])){
                int val = (int)map[i*rows+j];
                if(val < min){
                    min = val;
                    index = i*rows+j;
                }
            }
        }
    }
    //mexPrintf("Min %d, index %d\n", min, index);
    return index;
}

/* The gateway function */
void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray *prhs[])
{
    double *map;               /* 1xN input matrix */
    int rows, cols;                   /* size of matrix */
    int i, j, unknown;
    double *r_map;              /* output matrix */
    int *path_map;
    
    
    /* check for proper number of arguments */
    if(nrhs!=6) {
        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:nrhs","6 inputs required.");
    }
    if(nlhs!=2) {
        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:nlhs","2 outputs required.");
    }
    
    //map
    map = mxGetPr(prhs[0]);
    
    //rows of map
    rows = mxGetScalar(prhs[1]);
    //cols of map
    cols = mxGetScalar(prhs[2]);
    
    //start pose y
    i = mxGetScalar(prhs[3]);
    //start pose x
    j = mxGetScalar(prhs[4]);
    //plan through unknown territory
    unknown = mxGetScalar(prhs[5]);
    
    /* create the output matrix */
    plhs[0] = mxCreateDoubleMatrix(rows,cols,mxREAL);
    
    plhs[1] = mxCreateNumericMatrix(rows, cols, mxINT32_CLASS, mxREAL);
    
    /* get a pointer to the real data in the output matrix */
    r_map = mxGetPr(plhs[0]);
    
    path_map = (int*)mxGetPr(plhs[1]);
    
    /* call the computational routine */
    dijkstra(map, rows, cols, i, j, unknown, r_map, path_map);
}

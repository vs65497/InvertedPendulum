struct matrix {
    unsigned int rows, cols;
    double elements[4][4];
};

void matshow(matrix *a) {
    unsigned int i, j;
    for(i=0; i<a->rows;i++) {
        for(j=0; j<a->cols;j++) {
            Serial.println(a->elements[i][j], 4);
        }
    }
}

matrix matadd(matrix *a, matrix *b) {
    struct matrix resultant;
    resultant.rows = a->rows;
    resultant.cols = a->cols;

    if(a->rows != b->rows && a->cols != b->cols) {
        Serial.println("Matrix A and B not compatible");
        return resultant;
    }

    unsigned int i, j;
    for(i=0; i<a->rows;i++) {
        for(j=0; j<a->rows;j++) {
            resultant.elements[i][j] = a->elements[i][j] + b->elements[i][j];
        }
    }

    return resultant;
}

matrix matsub(matrix *a, matrix *b) {
    struct matrix resultant;
    resultant.rows = a->rows;
    resultant.cols = a->cols;

    if(a->rows != b->rows && a->cols != b->cols) {
        Serial.println("Matrix A and B not compatible");
        return resultant;
    }

    unsigned int i, j;
    for(i=0; i<a->rows;i++) {
        for(j=0; j<a->cols; j++) {
            resultant.elements[i][j] = a->elements[i][j] - b->elements[i][j];
        }
    }

    return resultant;
}

matrix matmul(matrix *a, matrix *b) {
    struct matrix resultant;
    resultant.rows = a->rows;
    resultant.cols = b->cols;

    if(a->cols != b->rows) {
        Serial.println("Matrix A and B not compatible");
        return resultant;
    }

    unsigned int i, j, k;
    double sum;
    for(i=0;i<4;i++) {    
        for(j=0;j<4;j++) { 
            resultant.elements[i][j] = 0;

            for(k=0;k<4;k++) {
                resultant.elements[i][j] += a->elements[i][k] * b->elements[k][j];    
            }    
        }    
    } 

    return resultant;
}

matrix matscale(double factor, matrix *a) {
    struct matrix resultant;
    resultant.rows = a->rows;
    resultant.cols = a->cols;

    unsigned int i, j;
    double value;
    for(i=0; i<a->rows;i++) {
        for(j=0; j<a->cols;j++) {
            resultant.elements[i][j] = factor * a->elements[i][j];
        }
    }

    return resultant;
}

struct matrix ref;
struct matrix x;
struct matrix xhat;
struct matrix u;
struct matrix y;
struct matrix abk;
struct matrix k;
struct matrix bmat;
struct matrix cmat;
struct matrix akfc;
struct matrix kf;
struct matrix chat;
struct matrix senserr;

void setStateSpace(double xpos, double th, double refx, double refth) {
    senserr.rows = 4;
    senserr.cols = 1;
    senserr.elements[0][0] = 0;
    senserr.elements[1][0] = 0;
    senserr.elements[2][0] = 0;
    senserr.elements[3][0] = 0;
  
    u.rows = 1;
    u.cols = 1;
    u.elements[0][0] = 0;
  
    bmat.rows = 4;
    bmat.cols = 1;
    bmat.elements[0][0] = 0;
    bmat.elements[0][1] = 3.24675;
    bmat.elements[0][2] = 0;
    bmat.elements[0][3] = 17.45566;
  
    cmat.rows = 1;
    cmat.cols = 4;
    cmat.elements[0][0] = 1;
    cmat.elements[0][1] = 1;
    cmat.elements[0][2] = 1;
    cmat.elements[0][3] = 1;

    chat.rows = 1;
    chat.cols = 4;
    chat.elements[0][0] = 1;
    chat.elements[0][1] = 1;
    chat.elements[0][2] = 1;
    chat.elements[0][3] = 1;
  
    ref.rows = 4;
    ref.cols = 1;
    ref.elements[0][0] = refx;
    ref.elements[1][0] = 0;
    ref.elements[2][0] = refth;
    ref.elements[3][0] = 0;

    x.rows = 4;
    x.cols = 1;
    x.elements[0][0] = xpos;
    x.elements[1][0] = 0;
    x.elements[2][0] = th;
    x.elements[3][0] = 0;

    xhat.rows = 4;
    xhat.cols = 1;
    xhat.elements[0][0] = xpos;
    xhat.elements[1][0] = 0;
    xhat.elements[2][0] = th;
    xhat.elements[3][0] = 0;

    y.rows = 4;
    y.cols = 1;
    y.elements[0][0] = xpos;
    y.elements[1][0] = 0;
    y.elements[2][0] = th;
    y.elements[3][0] = 0;

    abk.rows = 4;
    abk.cols = 4;
    abk.elements[0][0] = 0;
    abk.elements[0][1] = 1;
    abk.elements[0][2] = 0;
    abk.elements[0][3] = 0;

    abk.elements[1][0] =  0.0530 * 1000;
    abk.elements[1][1] =  0.0585 * 1000;
    abk.elements[1][2] = -0.3048 * 1000;
    abk.elements[1][3] = -0.0605 * 1000;

    abk.elements[2][0] =  0;
    abk.elements[2][1] =  0;
    abk.elements[2][2] = 0;
    abk.elements[2][3] = 1;

    abk.elements[3][0] =  0.2850 * 1000;
    abk.elements[3][1] =  0.3147 * 1000;
    abk.elements[3][2] = -1.5989 * 1000;
    abk.elements[3][3] = -0.3253 * 1000;

    k.rows = 1;
    k.cols = 4;
    k.elements[0][0] = -16.3299;
    k.elements[0][1] = -18.3311;
    k.elements[0][2] = 95.0855;
    k.elements[0][3] = 18.6358;

akfc.rows = 4;
akfc.cols = 4;

akfc.elements[0][0] = 1.682238;
akfc.elements[0][1] = 1.000000;
akfc.elements[0][2] = 1.682238;
akfc.elements[0][3] = 0.000000;

akfc.elements[1][0] = -7.948103;
akfc.elements[1][1] = -0.974026;
akfc.elements[1][2] = -4.030473;
akfc.elements[1][3] = 0.000000;

akfc.elements[2][0] = -22.615873;
akfc.elements[2][1] = 0.000000;
akfc.elements[2][2] = -22.615873;
akfc.elements[2][3] = 1.000000;

akfc.elements[3][0] = -183.660438;
akfc.elements[3][1] = -5.236699;
akfc.elements[3][2] = -109.855976;
akfc.elements[3][3] = 0.000000;

kf.rows = 4;
kf.cols = 1;

kf.elements[0][0] = -1.682238;
kf.elements[1][0] = 7.948103;
kf.elements[2][0] = 22.615873;
kf.elements[3][0] = 183.660438;

}

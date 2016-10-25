#ifndef ARM_MATH_H
#define ARM_MATH_H

#include <stdint.h>


// fixed point math library


#define MAX_VECTOR 4
#define FRACTION 256
#define FIXED(x) ((int)((x) * FRACTION))

//#define absf(x) ((x) > 0 ? (x) : (-(x)))
//#define abs(x) ((x) > 0 ? (x) : (-(x)))
#define abs_fixed(x) ((x) > 0 ? (x) : (-(x)))
extern const int16_t sin_table[];

typedef struct
{
	int rows;
	int columns;
	int values[MAX_VECTOR][MAX_VECTOR];
} matrix_t;


typedef struct
{
	int size;
	union
	{
		struct
		{
			int values[MAX_VECTOR];
		};
		struct
		{
			int x, y, z, w;
		};
		struct
		{
			int p, q, r;
		};
		struct
		{
			int a, b, c, d;
		};
	};
} vector_t;



// This structure automatically calculates derivatives for us
typedef struct
{
	int index;
	int total;
	int prev[0];
} derivative_t;

// Zero derivative structure
void init_derivative(derivative_t *ptr, int buffer_size);
// Push new value on derivative buffer
void update_derivative(derivative_t *ptr, int value);
// Get derivative
int get_derivative(derivative_t *ptr);
// get sum of all values
int get_sum(derivative_t *ptr);
void reset_derivative(derivative_t *ptr);
#define derivative_size(x) ((x)->total)





// For creating a new vector
#define NEW_VECTOR(name, size_) \
vector_t (name); \
(name).size = (size_);

#define INIT_VECTOR(name, size_) \
(name).size = (size_);

// For creating a new matrix
#define NEW_MATRIX(name, rows_, columns_) \
matrix_t (name); \
(name).rows = (rows_); \
(name).columns = (columns_);

#define INIT_MATRIX(name, rows_, columns_) \
(name).rows = (rows_); \
(name).columns = (columns_);

#define VECTOR_ENTRY(vector, entry) ((vector).values[(entry)])

#define MATRIX_ROW(matrix, row) ((matrix).values[(row)])
#define MATRIX_ENTRY(matrix, row, column) ((matrix).values[(row)][(column)])


void print_matrix(matrix_t *mat);
void print_vector(vector_t *vec);
// angles are -180 to 180
int get_angle_change(int old_angle, int new_angle);
int get_angle_change_fixed(int old_angle, int new_angle);
void sin_index(int *sign, int *index, int angle);
int sin_fixed(int angle);
// higher precision for the motor table
int sin_fixed14(int angle);
int cos_fixed(int angle);
int tan_fixed(int angle);
// sqrt from 0-64 * 256
int sqrt_fixed(int value);
int fix_angle(int angle);
void distance_angle(int *distance, 
	int *angle, 
	int x1,
	int y1,
	int x2, 
	int y2);
void multiply_matrix_vector(vector_t *dst, matrix_t *mat, vector_t *vec);


#endif


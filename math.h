#ifndef MATH_H
#define MATH_H



#define MAX_VECTOR 4
#define FRACTION 256

//#define absf(x) ((x) > 0 ? (x) : (-(x)))
//#define abs(x) ((x) > 0 ? (x) : (-(x)))
#define abs_fixed(x) ((x) > 0 ? (x) : (-(x)))

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
	int *prev;
	int index;
	int total;
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


// For creating a new matrix
#define NEW_MATRIX(name, rows_, columns_) \
matrix_t (name); \
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


#endif


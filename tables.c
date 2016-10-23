// construct tables

#include "stdio.h"
#include "stdlib.h"
#include <math.h>
#include "stdint.h"

#define N_SIN 256
#define MAX_PWM 65535
int table[N_SIN];

#define MAX_VECTOR 4

typedef struct
{
	int rows;
	int columns;
	float values[MAX_VECTOR][MAX_VECTOR];
} matrix_t;


typedef struct
{
	int size;
	union
	{
		struct
		{
			float values[MAX_VECTOR];
		};
		struct
		{
			float x, y, z, w;
		};
		struct
		{
			float p, q, r;
		};
		struct
		{
			float a, b, c, d;
		};
	};
} vector_t;

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

#define SQR(x) ((x) * (x))

void unity_matrix(matrix_t *matrix)
{
	int i, j;
	for(i = 0; i < matrix->rows; i++)
	{
		for(j = 0; j < matrix->columns; j++)
		{
			MATRIX_ENTRY(*matrix, i, j) = 0;
		}
	}
	
	for(i = 0; i < matrix->rows; i++)
	{
		MATRIX_ENTRY(*matrix, i, i) = 1;
	}
}

void copy_vector(vector_t *dst, vector_t *src)
{
	int i;
	for(i = 0; i < src->size; i++)
	{
		VECTOR_ENTRY(*dst, i) = VECTOR_ENTRY(*src, i);
	}
}

void subtract_vectors(vector_t *dst, vector_t *src1, vector_t *src2)
{
	int i;
	for(i = 0; i < src1->size; i++)
	{
		VECTOR_ENTRY(*dst, i) = VECTOR_ENTRY(*src1, i) - VECTOR_ENTRY(*src2, i);
	}
}

float vector_dot(vector_t *arg1, vector_t *arg2)
{
	int i;
	float result = 0;

	for(i = 0; i < arg1->size; i++)
		result += VECTOR_ENTRY(*arg1, i) * VECTOR_ENTRY(*arg2, i);
	return result;
}

void vector_cross(vector_t *dst, vector_t *arg1, vector_t *arg2)
{
	int i;
	dst->x = arg1->y * arg2->z - arg1->z * arg2->y;
	dst->y = arg1->z * arg2->x - arg1->x * arg2->z;
	dst->z = arg1->x * arg2->y - arg1->y * arg2->x;
}


void multiply_matrix_vector(vector_t *dst, matrix_t *mat, vector_t *vec)
{
	int i, j;
	int n = mat->rows;
	int m = mat->columns;

	if(m != vec->size || dst->size != n)
	{
		printf("multiply_matrix_vector: size mismatch");
		return;
	}

	for(i = 0; i < n; i++)
	{
		float s = 0;
		for(j = 0; j < m; j++)
			s += MATRIX_ENTRY(*mat, i, j) * VECTOR_ENTRY(*vec, j);
		VECTOR_ENTRY(*dst, i) = s;
	}
}


int pwm_table()
{
	int i;
	for(i = 0; i < N_SIN; i++)
	{
      table[i] = MAX_PWM / 2.0 + sin(2.0 * i / N_SIN * M_PI) * MAX_PWM / 2.0;
	}

	printf("const uint16_t sin_table[] = {\n\t");
	for(i = 0; i < N_SIN; i++)
	{
		printf("0x%04x", table[i]);
		if(i < N_SIN - 1) printf(", ");
		if(!((i + 1) % 16) && i != N_SIN - 1) printf("\n\t");
	}
	printf("\n};\n\n");
}


int sin_table()
{
	int i;
	printf("static const int16_t sin_table[] = {\n\t");
	for(i = 0; i < 256; i++)
	{
		printf("0x%04x", (int)(sin((float)i / 256 * (M_PI / 2)) * 0x4000));
		if(i < 255) printf(", ");
		if(!((i + 1) % 8) && i < 255) printf("\n\t");
	}
	printf("\n};\n");
}

// give the handle angle based on roll & pitch motors
float calculate_handle(float roll, float pitch)
{
	NEW_MATRIX(roll_matrix, 3, 3);
	NEW_MATRIX(pitch_matrix, 3, 3);
	NEW_VECTOR(point1, 3);
	NEW_VECTOR(point2, 3);
	NEW_VECTOR(point3, 3);

	NEW_VECTOR(result1, 3);
	NEW_VECTOR(result2, 3);
	NEW_VECTOR(result3, 3);

	unity_matrix(&roll_matrix);
	unity_matrix(&pitch_matrix);
	
	point1.x = -1;
	point1.y = 0;
	point1.z = -1;

	point2.x = 0;
	point2.y = 0;
	point2.z = 0;

	point3.x = 1;
	point3.y = 0;
	point3.z = -1;


	MATRIX_ENTRY(pitch_matrix, 1, 1) = cos(pitch);
	MATRIX_ENTRY(pitch_matrix, 1, 2) = sin(pitch);
	MATRIX_ENTRY(pitch_matrix, 2, 1) = -sin(pitch);
	MATRIX_ENTRY(pitch_matrix, 2, 2) = cos(pitch);

	MATRIX_ENTRY(roll_matrix, 0, 0) = cos(roll);
	MATRIX_ENTRY(roll_matrix, 0, 1) = sin(roll);
	MATRIX_ENTRY(roll_matrix, 1, 0) = -sin(roll);
	MATRIX_ENTRY(roll_matrix, 1, 1) = cos(roll);

	multiply_matrix_vector(&result1, &pitch_matrix, &point1);
	multiply_matrix_vector(&result2, &pitch_matrix, &point2);
	multiply_matrix_vector(&result3, &pitch_matrix, &point3);


	copy_vector(&point1, &result1);
	copy_vector(&point2, &result2);
	copy_vector(&point3, &result3);

	multiply_matrix_vector(&result1, &roll_matrix, &point1);
	multiply_matrix_vector(&result2, &roll_matrix, &point2);
	multiply_matrix_vector(&result3, &roll_matrix, &point3);

/*
 * 	printf("handle_table %d %f %f %f\n", 
 * 		__LINE__, 
 * 		VECTOR_ENTRY(result1, 0),
 * 		VECTOR_ENTRY(result1, 1),
 * 		VECTOR_ENTRY(result1, 2));
 */

// get the line normal to the 3 points
	NEW_VECTOR(plane_vector1, 3);
	NEW_VECTOR(plane_vector2, 3);
	subtract_vectors(&plane_vector1, &result2, &result1);
	subtract_vectors(&plane_vector2, &result2, &result3);

	NEW_VECTOR(plane_normal, 3);
	vector_cross(&plane_normal, &plane_vector1, &plane_vector2);

	NEW_VECTOR(y_axis, 3);
	y_axis.x = 0;
	y_axis.y = -1;
	y_axis.z = 0;
	
/*
 * 	printf("handle_table %d plane_vector1=%f %f %f\n", 
 * 		__LINE__, 
 * 		VECTOR_ENTRY(plane_vector1, 0),
 * 		VECTOR_ENTRY(plane_vector1, 1),
 * 		VECTOR_ENTRY(plane_vector1, 2));
 * 	printf("handle_table %d plane_vector2=%f %f %f\n", 
 * 		__LINE__, 
 * 		VECTOR_ENTRY(plane_vector2, 0),
 * 		VECTOR_ENTRY(plane_vector2, 1),
 * 		VECTOR_ENTRY(plane_vector2, 2));
 * 	printf("handle_table %d plane_normal=%f %f %f\n", 
 * 		__LINE__, 
 * 		VECTOR_ENTRY(plane_normal, 0),
 * 		VECTOR_ENTRY(plane_normal, 1),
 * 		VECTOR_ENTRY(plane_normal, 2));
 */

	
// angle between normal & y axis
	float result = acos(vector_dot(&plane_normal, &y_axis) /
		(sqrt(SQR(plane_normal.x) + SQR(plane_normal.y) + SQR(plane_normal.z)) *
		sqrt(SQR(y_axis.x) + SQR(y_axis.y) + SQR(y_axis.z))));
//	printf("handle_table %d %f\n", __LINE__, result * 360 / 2 / M_PI);	

	return result;
}

// correlate roll & pitch motors to handle angle
void handle_table()
{
// 	double pitch = 0;
//	double roll = 45 * 2 * M_PI / 360;
//	double pitch = 45 * 2 * M_PI / 360;
//	double roll = 0;
//	double roll = -14 * 2 * M_PI / 360;
//	double pitch = 55 * 2 * M_PI / 360;

	printf("static const unsigned char handle_table[] = {\n");
	float roll, pitch;
// negative roll is the mirror of positive roll
	for(roll = 0; roll <= 45; roll += 5)
	{
// handle is always 90 when pitch is 90
// handle is always roll when pitch is 0
		for(pitch = 5; pitch < 90; pitch += 5)
		{
			float handle = calculate_handle(roll * 2 * M_PI / 360, 
				pitch * 2 * M_PI / 360);
			
			if(pitch <= 8)
			{
				printf("    ");
			}
			printf("%.0f", handle * 360 / 2 / M_PI);
			if(pitch >= 83)
			{
				if(roll < 45)
				{
					printf(",");
				}
				printf(" // roll %.0f\n", roll);
			}
			else
			{
				printf(", ");
			}
		}
	}
	printf("\n};\n");
}



int main()
{
//	sin_table();
	handle_table();
}


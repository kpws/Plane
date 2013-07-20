/* QR Least-Squares Solver with Column Pivoting
 * 
 * Note: This code will overwrite the input matrix! Create a copy if needed.
 *
 * Implemented in ANSI C. To compile with gcc: gcc -o qr_fact qr_fact.c -lm
 *
 * Author: Michael Mazack, <mazack @ yahoo . com>
 *
 * Date: April 27th, 2010
 *
 * License: Public Domain. Redistribution and modification without 
 * restriction is granted. If you find this code helpful, please let me know.
 */

#include <math.h>

/* For updating the permuatation vector in (virtual) column swaps. */
void swap_cols(unsigned int *p, unsigned int i, unsigned int j);

/* Backsolving of a trianglular system. */
void back_solve(double **mat, double *rhs, unsigned int rows,
		unsigned int cols, double *sol, unsigned int *p);

/* Apply a Householder transform to the matrix at a given spot. */
void householder(double **mat, unsigned int rows, unsigned int cols,
		 unsigned int row_pos, unsigned int col_pos, double *result);

/* Routine for applying the Householder transform to the matrix and the 
 * right hand side. */
void apply_householder(double **mat, double *rhs, unsigned int rows, 
		       unsigned int cols, double *house, unsigned int row_pos,
		       unsigned int *p);

/* Get the column with the largest sub-norm starting from i = p[j] = row_pos. */
int get_next_col(double **mat, unsigned int rows, unsigned int cols,
			  unsigned int row_pos, unsigned int *p);

/* Solve the least squares problem, sol = mat\rhs . */
void qr_least_squares(double **mat, double *rhs, double *sol, 
		      unsigned int rows, unsigned int cols);

/* A simple matrix-vector product routine. */
void mat_vec(double **mat, unsigned int rows, unsigned int cols,
	     double *vec, double *rhs);

/* Routine for displaying a matrix. */
void display_mat(double **mat, unsigned int rows, unsigned int cols,
		 unsigned int *p);

/* Routine for displaying a vector. */
void display_vec(double *vec, unsigned int rows, unsigned int *p);

void swap_cols(unsigned int *p, unsigned int i, unsigned int j)
{
  unsigned int temp;
  temp = p[i];
  p[i] = p[j];
  p[j] = temp;
}

void back_solve(double **mat, double *rhs, unsigned int rows,
		unsigned int cols, double *sol, unsigned int *p)
{
  int i, j, bottom;
  double sum;

  /* Fill the solution with zeros initially. */
  for(i = 0; i < cols; i++)
    sol[i] = 0.0;

  /* Find the first non-zero row from the bottom and start solving from here. */
  for(i = rows - 1; i >= 0; i--)
    if(fabs(mat[i][p[cols - 1]]) > 1e-7)
      {
	bottom = i;
	break;
      }

  /* Standard back solving routine starting at the first non-zero diagonal. */
  for(i = bottom; i >= 0; i--)
    {
      sum = 0.0;

      for(j = cols - 1; j >= 0; j--)
	if(j > i)
	  sum += sol[p[j]]*mat[i][p[j]];
      
      if(mat[i][p[i]] > 1e-7)
	sol[p[i]] = (rhs[i] - sum)/mat[i][p[i]];
      else
	sol[p[i]] = 0.0;
    }
}

void householder(double **mat, unsigned int rows, unsigned int cols,
		 unsigned int row_pos, unsigned int col_pos, double *result)
{
  int i;
  double norm;

  norm = 0;
  for(i = row_pos; i < rows; i++)
    norm += mat[i][col_pos]*mat[i][col_pos];

  if(norm == 0)
    return;

  norm = sqrt(norm);

  result[0] = (mat[row_pos][col_pos] - norm);

  for(i = 1; i < (rows - row_pos); i++)
    result[i] = mat[i+row_pos][col_pos];

  norm = 0;
  for(i = 0; i < (rows - row_pos); i++)
    norm += result[i]*result[i];

  if(norm == 0)
    return;

  norm = sqrt(norm);

  for(i = 0; i < (rows - row_pos); i++)
    result[i] *= (1.0/norm);
}

void apply_householder(double **mat, double *rhs, unsigned int rows, unsigned int cols, double *house, unsigned int row_pos, unsigned int *p)
{
  int i, j, k, n;
  double sum;
  double **hhmat;
  double **mat_cpy;
  double *rhs_cpy;

  // Get the dimensions for the Q matrix.
  n = rows - row_pos;

  // Allocate memory.
  hhmat = malloc(sizeof(double *)*n);
  for(i = 0; i < n; i++)
    hhmat[i] = malloc(sizeof(double)*n);

  mat_cpy = malloc(sizeof(double *)*rows);
  for(i = 0; i < rows; i++)
    mat_cpy[i] = malloc(sizeof(double)*cols);

  rhs_cpy = malloc(sizeof(double )*rows);
  
  // Copy the matrix.
  for(i = 0; i < rows; i++)
    for(j = 0; j < cols; j++)
      mat_cpy[i][j] = mat[i][j];

  // Copy the right hand side.
  for(i = 0; i < rows; i++)
    rhs_cpy[i] = rhs[i];

  // Build the Q matrix from the Householder transform.
  for(j = 0; j < n; j++)
    for(i = 0; i < n; i++)
      if(i != j)
	hhmat[i][j] = -2.0*house[j]*house[i];
      else
	hhmat[i][j] = 1.0 - 2.0*house[j]*house[i];

  // Multiply by the Q matrix.
  for(k = 0; k < cols; k++)
    for(j = 0; j < n; j++)
      {
	sum = 0.0;
	for(i = 0; i < n; i++)
	  sum += hhmat[j][i]*mat_cpy[i + row_pos][p[k]];
	  
	mat[j + row_pos][p[k]] = sum;
      }

  // Multiply the rhs by the Q matrix.
  for(j = 0; j < n; j++)
    {
      sum = 0.0;
      for(i = 0; i < n; i++)
	sum += hhmat[i][j]*rhs_cpy[i + row_pos];

      rhs[j + row_pos] = sum;
    }

  // Collect garbage.
  for(i = 0; i < (rows - row_pos); i++)
    free(hhmat[i]);

  for(i = 0; i < rows; i++)
    free(mat_cpy[i]);

  free(hhmat);
  free(mat_cpy);
  free(rhs_cpy);
}

int get_next_col(double **mat, unsigned int rows, unsigned int cols,
			  unsigned int row_pos, unsigned int *p)
{
  int i, j, max_loc;
  double *col_norms;
  double max;

  max_loc = -1;
  col_norms = malloc(sizeof(double)*cols);

  // Compute the norms of the sub columns.
  for(j = 0; j < cols; j++)
    {
      col_norms[j] = 0;

      for(i = row_pos; i < rows; i++)
	col_norms[j] += mat[i][p[j]]*mat[i][p[j]];
    }

  // Find the maximum location.
  max = 1e-7;
  for(i = 0; i < cols; i++)
    if(col_norms[i] > max)
      {
	max = col_norms[i];
	max_loc = i;
      }

  // Collect garbge and return.
  free(col_norms);  
  return max_loc;
}

/* The star of the show. A QR least-squares solving routine for x = A\b.
 *
 * First argument : The row-major matrix (2D array), A.
 * Second argument: The right-hand side vector, b.
 * Third argument : The solution vector, x.
 * Fourth argument: The number of rows in A.
 * Fifth argument : The number of columns in A.
 *
 * WARNING: This routine will overwrite the matrix A and the right-hand side
 * vector b. In other words, A*x = b is solved using QR least-squares with, 
 * column pivoting, but neither the A nor b are what you started with. However,
 * the solution x corresponds to the solution of both the modified and original
 * systems. Please be aware of this.
 */
void qr_least_squares(double **mat, double *rhs, double *sol, 
		      unsigned int rows, unsigned int cols)
{
  int i, max_loc;
  unsigned int *p;
  double *v;

  /* Allocate memory for index vector and Householder transform vector. */
  p = malloc(sizeof(unsigned int)*cols);
  v = malloc(sizeof(double)*rows);

  /* Initial permutation vector. */
  for(i = 0; i < cols; i++)
    p[i] = i;
  
  /* Apply rotators to make R and Q'*b */
  for(i = 0; i < cols; i++)
    {
      max_loc = get_next_col(mat, rows, cols, i, p);
      if(max_loc >= 0)
	swap_cols(p, i, max_loc);

      householder(mat, rows, cols, i, p[i], v);
      apply_householder(mat, rhs, rows, cols, v, i, p);
    }

  /* Back solve Rx = Q'*b */
  back_solve(mat, rhs, rows, cols, sol, p);

  /* Collect garbage. */
  free(p);
  free(v);
}

/* A very simple matrix vector product routine. */
void mat_vec(double **mat, unsigned int rows, unsigned int cols,
	     double *vec, double *rhs)
{
  int i, j;
  double sum;

  for(i =  0; i < rows; i++)
    {
      sum = 0.0;
      for(j = 0; j < cols; j++)
	sum += mat[i][j]*vec[j];

      rhs[i] = sum;
    }
}

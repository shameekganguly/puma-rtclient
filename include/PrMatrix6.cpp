// *******************************************************************
// PrMatrix6.cpp
//
// Implementation of a 6x6 matrix.
//
// modification history
// --------------------
//
// 06/17/04: Dan Merget: Made into a subclass of PrMatrix
// 11/20/97: K.C. Chang: added inline methods.
// 11/05/97: K.C. Chang: created.
// *******************************************************************

#include "PrMatrix6.h"

const PrMatrix6 PrMatrix6::IDENTITY(PrMatrix3(1, 0, 0,   0, 1, 0,   0, 0, 1),
                                    PrMatrix3(0, 0, 0,   0, 0, 0,   0, 0, 0),
                                    PrMatrix3(0, 0, 0,   0, 0, 0,   0, 0, 0),
                                    PrMatrix3(1, 0, 0,   0, 1, 0,   0, 0, 1));
const PrMatrix6 PrMatrix6::ZERO;

// ===================================================================
// Destructor: In C++, the child destructor is called before the
// base-class destructor.  So by setting m_data back to NULL here, we
// ensure that the base class won't try to delete m_buff.
// ===================================================================
PrMatrix6::~PrMatrix6()
{
  m_data = NULL;
  m_row  = 0;
  m_col  = 0;
  m_size = 0;
}

// ===================================================================
// setDiagonal(): Set the diagonal elements
// ===================================================================
void PrMatrix6::setDiagonal( Float d0, Float d1, Float d2,
                             Float d3, Float d4, Float d5 )
{
  m_buff[0][0] = d0;
  m_buff[1][1] = d1;
  m_buff[2][2] = d2;
  m_buff[3][3] = d3;
  m_buff[4][4] = d4;
  m_buff[5][5] = d5;
}

PrVector6 PrMatrix6::diagonal() const
{
  return PrVector6( m_buff[0][0], m_buff[1][1], m_buff[2][2],
                    m_buff[3][3], m_buff[4][4], m_buff[5][5] );
}

void PrMatrix6::diagonal( PrVector& dest ) const
{
  dest.setSize( 6 );
  dest[0] = m_buff[0][0];
  dest[1] = m_buff[1][1];
  dest[2] = m_buff[2][2];
  dest[3] = m_buff[3][3];
  dest[4] = m_buff[4][4];
  dest[5] = m_buff[5][5];
}

void PrMatrix6::setDiagonal( Float src )
{
  m_buff[0][0] = src;
  m_buff[1][1] = src;
  m_buff[2][2] = src;
  m_buff[3][3] = src;
  m_buff[4][4] = src;
  m_buff[5][5] = src;
}

void PrMatrix6::setDiagonal( const Float* src )
{
  m_buff[0][0] = src[0];
  m_buff[1][1] = src[1];
  m_buff[2][2] = src[2];
  m_buff[3][3] = src[3];
  m_buff[4][4] = src[4];
  m_buff[5][5] = src[5];
}

void PrMatrix6::setDiagonal( const PrVector& src )
{
  SAIAssert( src.size() == 6 );
  m_buff[0][0] = src[0];
  m_buff[1][1] = src[1];
  m_buff[2][2] = src[2];
  m_buff[3][3] = src[3];
  m_buff[4][4] = src[4];
  m_buff[5][5] = src[5];
}

// ===================================================================
// getRow() / getColumn(): Same as the base-class method, but
// optimized for 6x6 matrices.
// ===================================================================
void PrMatrix6::getRow( int row, PrVector& dest ) const
{
  SAIAssert( row >= 0 && row < 6 );
  dest.setSize( 6 );
  dest[0] = m_buff[row][0];
  dest[1] = m_buff[row][1];
  dest[2] = m_buff[row][2];
  dest[3] = m_buff[row][3];
  dest[4] = m_buff[row][4];
  dest[5] = m_buff[row][5];
}

void PrMatrix6::getColumn( int col, PrVector& dest ) const
{
  SAIAssert( col >= 0 && col < 6 );
  dest.setSize( 6 );
  dest[0] = m_buff[0][col];
  dest[1] = m_buff[1][col];
  dest[2] = m_buff[2][col];
  dest[3] = m_buff[3][col];
  dest[4] = m_buff[4][col];
  dest[5] = m_buff[5][col];
}

// ===================================================================
// submatrix(): Same as the base-class method, but optimized for
// extracting 3x3 submatrices
// ===================================================================
void PrMatrix6::submatrix( int row, int col, PrMatrix& dest ) const
{
  if( dest.row() != 3 || dest.column() != 3 )
  {
    // If dest is not 3x3, use the generic base-class method
    PrMatrix::submatrix( row, col, dest );
  }
  else
  {
    // Copy a 3x3 submatrix
    SAIAssert( row >= 0 && row <= 3 );
    SAIAssert( col >= 0 && col <= 3 );
    const Float* src = &m_buff[row][col];

    dest[0][0] = src[6 * 0 + 0];
    dest[0][1] = src[6 * 0 + 1];
    dest[0][2] = src[6 * 0 + 2];

    dest[1][0] = src[6 * 1 + 0];
    dest[1][1] = src[6 * 1 + 1];
    dest[1][2] = src[6 * 1 + 2];

    dest[2][0] = src[6 * 2 + 0];
    dest[2][1] = src[6 * 2 + 1];
    dest[2][2] = src[6 * 2 + 2];
  }
}

// ===================================================================
// setSubmatrix(): Same as the base-class method, but optimized for
// copying 3x3 submatrices
// ===================================================================
void PrMatrix6::setSubmatrix( int row, int col, const PrMatrix& src )
{
  if( src.row() != 3 || src.column() != 3 )
  {
    // If src is not 3x3, use the generic base-class method
    PrMatrix::setSubmatrix( row, col, src );
  }
  else
  {
    // Copy a 3x3 submatrix
    SAIAssert( row >= 0 && row <= 3 );
    SAIAssert( col >= 0 && col <= 3 );
    Float* dest = &m_buff[row][col];

    dest[6 * 0 + 0] = src[0][0];
    dest[6 * 0 + 1] = src[0][1];
    dest[6 * 0 + 2] = src[0][2];
    dest[6 * 1 + 0] = src[1][0];
    dest[6 * 1 + 1] = src[1][1];
    dest[6 * 1 + 2] = src[1][2];
    dest[6 * 2 + 0] = src[2][0];
    dest[6 * 2 + 1] = src[2][1];
    dest[6 * 2 + 2] = src[2][2];
  }
}

// ===================================================================
// identity(): Same as the base-class method, but optimized for 6x6
// matrices.
// ===================================================================
void PrMatrix6::identity()
{
  zero();
  m_buff[0][0] = 1;
  m_buff[1][1] = 1;
  m_buff[2][2] = 1;
  m_buff[3][3] = 1;
  m_buff[4][4] = 1;
  m_buff[5][5] = 1;
}

// ===================================================================
// negate(): Same as the base-class method, but optimized for 6x6
// matrices.
// ===================================================================
void PrMatrix6::negate( PrMatrix& dest ) const
{
  dest.setSize( 6, 6 );
  for( int ii = 0; ii < 6; ii++ )
  {
    const Float* srcRow  = m_buff[ii];
    Float* destRow = dest[ii];

    destRow[0] = -srcRow[0];
    destRow[1] = -srcRow[1];
    destRow[2] = -srcRow[2];
    destRow[3] = -srcRow[3];
    destRow[4] = -srcRow[4];
    destRow[5] = -srcRow[5];
  }
}

// ===================================================================
// Arithmetic operations: Same as the base-class method, but optimized
// for 6x6 matrices.
// ===================================================================
void PrMatrix6::add( const PrMatrix& rhs, PrMatrix& dest ) const
{
  SAIAssert( rhs.row() == 6 && rhs.column() == 6 );
  dest.setSize( 6, 6 );
  for( int ii = 0; ii < 6; ii++ )
  {
    const Float* lhsRow = m_buff[ii];
    const Float* rhsRow = rhs[ii];
    Float* destRow = dest[ii];

    destRow[0] = lhsRow[0] + rhsRow[0];
    destRow[1] = lhsRow[1] + rhsRow[1];
    destRow[2] = lhsRow[2] + rhsRow[2];
    destRow[3] = lhsRow[3] + rhsRow[3];
    destRow[4] = lhsRow[4] + rhsRow[4];
    destRow[5] = lhsRow[5] + rhsRow[5];
  }
}

void PrMatrix6::add( const PrMatrix& rhs )
{
  SAIAssert( rhs.row() == 6 && rhs.column() == 6 );
  for( int ii = 0; ii < 6; ii++ )
  {
    Float* lhsRow = m_buff[ii];
    const Float* rhsRow = rhs[ii];

    lhsRow[0] += rhsRow[0];
    lhsRow[1] += rhsRow[1];
    lhsRow[2] += rhsRow[2];
    lhsRow[3] += rhsRow[3];
    lhsRow[4] += rhsRow[4];
    lhsRow[5] += rhsRow[5];
  }
}

void PrMatrix6::subtract( const PrMatrix& rhs, PrMatrix& dest ) const
{
  SAIAssert( rhs.row() == 6 && rhs.column() == 6 );
  dest.setSize( 6, 6 );
  for( int ii = 0; ii < 6; ii++ )
  {
    const Float* lhsRow = m_buff[ii];
    const Float* rhsRow = rhs[ii];
    Float* destRow = dest[ii];

    destRow[0] = lhsRow[0] - rhsRow[0];
    destRow[1] = lhsRow[1] - rhsRow[1];
    destRow[2] = lhsRow[2] - rhsRow[2];
    destRow[3] = lhsRow[3] - rhsRow[3];
    destRow[4] = lhsRow[4] - rhsRow[4];
    destRow[5] = lhsRow[5] - rhsRow[5];
  }
}

void PrMatrix6::subtract( const PrMatrix& rhs )
{
  SAIAssert( rhs.row() == 6 && rhs.column() == 6 );
  for( int ii = 0; ii < 6; ii++ )
  {
    Float* lhsRow = m_buff[ii];
    const Float* rhsRow = rhs[ii];

    lhsRow[0] -= rhsRow[0];
    lhsRow[1] -= rhsRow[1];
    lhsRow[2] -= rhsRow[2];
    lhsRow[3] -= rhsRow[3];
    lhsRow[4] -= rhsRow[4];
    lhsRow[5] -= rhsRow[5];
  }
}

void PrMatrix6::multiply( const PrMatrix& rhs, PrMatrix& dest,
                                               bool fTranspose ) const
{
  if ( rhs.row() != 6 || rhs.column() != 6 )
  {
    // If rhs is not 6x6, then just use the generic parent-class method
    PrMatrix::multiply( rhs, dest, fTranspose );
  }
  else if ( !fTranspose )
  {
    // dest = this * rhs, where rhs is 6x6
    SAIAssert( &dest != static_cast<const PrMatrix*>(this) && &dest != &rhs );
    dest.setSize( 6, 6 );
    for( int ii = 0; ii < 6; ii++ )
    {
      const Float* a = m_buff[ii];
      const PrMatrix& b = rhs;
      Float* destRow = dest[ii];
      destRow[0] = a[0] * b[0][0] + a[1] * b[1][0] + a[2] * b[2][0] +
                   a[3] * b[3][0] + a[4] * b[4][0] + a[5] * b[5][0];
      destRow[1] = a[0] * b[0][1] + a[1] * b[1][1] + a[2] * b[2][1] +
                   a[3] * b[3][1] + a[4] * b[4][1] + a[5] * b[5][1];
      destRow[2] = a[0] * b[0][2] + a[1] * b[1][2] + a[2] * b[2][2] +
                   a[3] * b[3][2] + a[4] * b[4][2] + a[5] * b[5][2];
      destRow[3] = a[0] * b[0][3] + a[1] * b[1][3] + a[2] * b[2][3] +
                   a[3] * b[3][3] + a[4] * b[4][3] + a[5] * b[5][3];
      destRow[4] = a[0] * b[0][4] + a[1] * b[1][4] + a[2] * b[2][4] +
                   a[3] * b[3][4] + a[4] * b[4][4] + a[5] * b[5][4];
      destRow[5] = a[0] * b[0][5] + a[1] * b[1][5] + a[2] * b[2][5] +
                   a[3] * b[3][5] + a[4] * b[4][5] + a[5] * b[5][5];
    }
  }
  else
  {
    // dest = this * rhs^T, where rhs is 6x6
    SAIAssert( &dest != static_cast<const PrMatrix*>(this) && &dest != &rhs );
    dest.setSize( 6, 6 );
    for( int ii = 0; ii < 6; ii++ )
    {
      const Float* a = m_buff[ii];
      const PrMatrix& b = rhs;
      Float* destRow = dest[ii];
      destRow[0] = a[0] * b[0][0] + a[1] * b[0][1] + a[2] * b[0][2]
                 + a[3] * b[0][3] + a[4] * b[0][4] + a[5] * b[0][5];
      destRow[1] = a[0] * b[1][0] + a[1] * b[1][1] + a[2] * b[1][2]
                 + a[3] * b[1][3] + a[4] * b[1][4] + a[5] * b[1][5];
      destRow[2] = a[0] * b[2][0] + a[1] * b[2][1] + a[2] * b[2][2]
                 + a[3] * b[2][3] + a[4] * b[2][4] + a[5] * b[2][5];
      destRow[3] = a[0] * b[3][0] + a[1] * b[3][1] + a[2] * b[3][2]
                 + a[3] * b[3][3] + a[4] * b[3][4] + a[5] * b[3][5];
      destRow[4] = a[0] * b[4][0] + a[1] * b[4][1] + a[2] * b[4][2]
                 + a[3] * b[4][3] + a[4] * b[4][4] + a[5] * b[4][5];
      destRow[5] = a[0] * b[5][0] + a[1] * b[5][1] + a[2] * b[5][2]
                 + a[3] * b[5][3] + a[4] * b[5][4] + a[5] * b[5][5];
    }
  }
}

void PrMatrix6::multiply( const PrVector& rhs, PrVector& dest ) const
{
  SAIAssert( &dest != &rhs );
  SAIAssert( rhs.size() == 6 );
  dest.setSize( 6 );
  const PrMatrix6& lhs = *this;

  dest[0] = lhs[0][0] * rhs[0] + lhs[0][1] * rhs[1] + lhs[0][2] * rhs[2]
          + lhs[0][3] * rhs[3] + lhs[0][4] * rhs[4] + lhs[0][5] * rhs[5];
  dest[1] = lhs[1][0] * rhs[0] + lhs[1][1] * rhs[1] + lhs[1][2] * rhs[2]
          + lhs[1][3] * rhs[3] + lhs[1][4] * rhs[4] + lhs[1][5] * rhs[5];
  dest[2] = lhs[2][0] * rhs[0] + lhs[2][1] * rhs[1] + lhs[2][2] * rhs[2]
          + lhs[2][3] * rhs[3] + lhs[2][4] * rhs[4] + lhs[2][5] * rhs[5];
  dest[3] = lhs[3][0] * rhs[0] + lhs[3][1] * rhs[1] + lhs[3][2] * rhs[2]
          + lhs[3][3] * rhs[3] + lhs[3][4] * rhs[4] + lhs[3][5] * rhs[5];
  dest[4] = lhs[4][0] * rhs[0] + lhs[4][1] * rhs[1] + lhs[4][2] * rhs[2]
          + lhs[4][3] * rhs[3] + lhs[4][4] * rhs[4] + lhs[4][5] * rhs[5];
  dest[5] = lhs[5][0] * rhs[0] + lhs[5][1] * rhs[1] + lhs[5][2] * rhs[2]
          + lhs[5][3] * rhs[3] + lhs[5][4] * rhs[4] + lhs[5][5] * rhs[5];
}

void PrMatrix6::multiply( Float rhs, PrMatrix& dest ) const
{
  dest.setSize( 6, 6 );
  for( int ii = 0; ii < 6; ii++ )
  {
    const Float* lhsRow = m_buff[ii];
    Float* destRow = dest[ii];

    destRow[0] = lhsRow[0] * rhs;
    destRow[1] = lhsRow[1] * rhs;
    destRow[2] = lhsRow[2] * rhs;
    destRow[3] = lhsRow[3] * rhs;
    destRow[4] = lhsRow[4] * rhs;
    destRow[5] = lhsRow[5] * rhs;
  }
}

void PrMatrix6::multiply( Float rhs )
{
  for( int ii = 0; ii < 6; ii++ )
  {
    Float* lhsRow = m_buff[ii];
    lhsRow[0] *= rhs;
    lhsRow[1] *= rhs;
    lhsRow[2] *= rhs;
    lhsRow[3] *= rhs;
    lhsRow[4] *= rhs;
    lhsRow[5] *= rhs;
  }
}

void PrMatrix6::multiplyTranspose( const PrMatrix& rhs, PrMatrix& dest ) const
{
  if ( rhs.row() != 6 || rhs.column() != 6 )
  {
    // If rhs is not 6x6, then just use the generic parent-class method
    PrMatrix::multiplyTranspose( rhs, dest );
  }
  else
  {
    // dest = this^T * rhs, where rhs is 6x6
    SAIAssert( &dest != static_cast<const PrMatrix*>(this) && &dest != &rhs );
    dest.setSize( 6, 6 );
    for( int ii = 0; ii < 6; ii++ )
    {
      const Float* a = &m_buff[0][ii];
      const PrMatrix& b = rhs;
      Float* destRow = dest[ii];
      destRow[0] = a[6*0] * b[0][0] + a[6*1] * b[1][0] + a[6*2] * b[2][0] +
                   a[6*3] * b[3][0] + a[6*4] * b[4][0] + a[6*5] * b[5][0];
      destRow[1] = a[6*0] * b[0][1] + a[6*1] * b[1][1] + a[6*2] * b[2][1] +
                   a[6*3] * b[3][1] + a[6*4] * b[4][1] + a[6*5] * b[5][1];
      destRow[2] = a[6*0] * b[0][2] + a[6*1] * b[1][2] + a[6*2] * b[2][2] +
                   a[6*3] * b[3][2] + a[6*4] * b[4][2] + a[6*5] * b[5][2];
      destRow[3] = a[6*0] * b[0][3] + a[6*1] * b[1][3] + a[6*2] * b[2][3] +
                   a[6*3] * b[3][3] + a[6*4] * b[4][3] + a[6*5] * b[5][3];
      destRow[4] = a[6*0] * b[0][4] + a[6*1] * b[1][4] + a[6*2] * b[2][4] +
                   a[6*3] * b[3][4] + a[6*4] * b[4][4] + a[6*5] * b[5][4];
      destRow[5] = a[6*0] * b[0][5] + a[6*1] * b[1][5] + a[6*2] * b[2][5] +
                   a[6*3] * b[3][5] + a[6*4] * b[4][5] + a[6*5] * b[5][5];
    }
  }
}

void PrMatrix6::multiplyTranspose( const PrVector& rhs, PrVector& dest ) const
{
  SAIAssert( &dest != &rhs );
  SAIAssert( rhs.size() == 6 );
  dest.setSize( 6 );
  const PrMatrix6& lhs = *this;

  dest[0] = lhs[0][0] * rhs[0] + lhs[1][0] * rhs[1] + lhs[2][0] * rhs[2]
          + lhs[3][0] * rhs[3] + lhs[4][0] * rhs[4] + lhs[5][0] * rhs[5];
  dest[1] = lhs[0][1] * rhs[0] + lhs[1][1] * rhs[1] + lhs[2][1] * rhs[2]
          + lhs[3][1] * rhs[3] + lhs[4][1] * rhs[4] + lhs[5][1] * rhs[5];
  dest[2] = lhs[0][2] * rhs[0] + lhs[1][2] * rhs[1] + lhs[2][2] * rhs[2]
          + lhs[3][2] * rhs[3] + lhs[4][2] * rhs[4] + lhs[5][2] * rhs[5];
  dest[3] = lhs[0][3] * rhs[0] + lhs[1][3] * rhs[1] + lhs[2][3] * rhs[2]
          + lhs[3][3] * rhs[3] + lhs[4][3] * rhs[4] + lhs[5][3] * rhs[5];
  dest[4] = lhs[0][4] * rhs[0] + lhs[1][4] * rhs[1] + lhs[2][4] * rhs[2]
          + lhs[3][4] * rhs[3] + lhs[4][4] * rhs[4] + lhs[5][4] * rhs[5];
  dest[5] = lhs[0][5] * rhs[0] + lhs[1][5] * rhs[1] + lhs[2][5] * rhs[2]
          + lhs[3][5] * rhs[3] + lhs[4][5] * rhs[4] + lhs[5][5] * rhs[5];
}

void PrMatrix6::transpose( PrMatrix& dest ) const
{
  SAIAssert( &dest != static_cast<const PrMatrix*>(this) );
  dest.setSize( 6, 6 );

  for( int ii = 0; ii < 6; ii++ )
  {
    const Float* srcCol = &m_buff[0][ii];
    Float* destRow = dest[ii];

    destRow[0] = srcCol[6 * 0];
    destRow[1] = srcCol[6 * 1];
    destRow[2] = srcCol[6 * 2];
    destRow[3] = srcCol[6 * 3];
    destRow[4] = srcCol[6 * 4];
    destRow[5] = srcCol[6 * 5];
  }
}

// ===================================================================
// display(): Display the matrix
// ===================================================================
void PrMatrix6::display( const char* name ) const
{
  if( name != NULL )
  {
    printf("%s =\n", name );
  }

  printf( "[%f %f %f %f %f %f]\n"
          "[%f %f %f %f %f %f]\n"
          "[%f %f %f %f %f %f]\n",
          m_buff[0][0], m_buff[0][1], m_buff[0][2],
          m_buff[0][3], m_buff[0][4], m_buff[0][5],
          m_buff[1][0], m_buff[1][1], m_buff[1][2],
          m_buff[1][3], m_buff[1][4], m_buff[1][5],
          m_buff[2][0], m_buff[2][1], m_buff[2][2],
          m_buff[2][3], m_buff[2][4], m_buff[2][5] );
  printf( "[%f %f %f %f %f %f]\n"
          "[%f %f %f %f %f %f]\n"
          "[%f %f %f %f %f %f]\n",
          m_buff[3][0], m_buff[3][1], m_buff[3][2],
          m_buff[3][3], m_buff[3][4], m_buff[3][5],
          m_buff[4][0], m_buff[4][1], m_buff[4][2],
          m_buff[4][3], m_buff[4][4], m_buff[4][5],
          m_buff[5][0], m_buff[5][1], m_buff[5][2],
          m_buff[5][3], m_buff[5][4], m_buff[5][5] );
}

// ===================================================================
// dynamic(): This virtual function tells the base that m_data was
// not dynamically allocated in this object.
// ===================================================================
bool PrMatrix6::dynamic() const
{
  return false;
}

// ===================================================================
// resize(): Virtual function that the base class uses to resize the
// matrix.  It is an error to resize a PrMatrix6 to any size other
// than 6x6.
// ===================================================================
void PrMatrix6::resize( int row, int col )
{
  SAIAssert( row == 6 && col == 6 );
}

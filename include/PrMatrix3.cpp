// *******************************************************************
// PrMatrix3.cpp
//
// Implementation of a 3x3 matrix.
//
// modification history
// --------------------
//
// 06/17/04: Dan Merget: Made into a subclass of PrMatrix
// 11/20/97: K.C. Chang: added inline methods.
// 11/05/97: K.C. Chang: created.
// *******************************************************************

#include "PrMatrix3.h"

const PrMatrix3 PrMatrix3::IDENTITY( 1, 0, 0,   0, 1, 0,   0, 0, 1 );
const PrMatrix3 PrMatrix3::ZERO;

// ===================================================================
// Destructor: In C++, the child destructor is called before the
// base-class destructor.  So by setting m_data back to NULL here, we
// ensure that the base class won't try to delete m_buff.
// ===================================================================
PrMatrix3::~PrMatrix3()
{
  m_data = NULL;
  m_row  = 0;
  m_col  = 0;
  m_size = 0;
}

// ===================================================================
// angularError(): Find the instantaneous angular error between this
// orientation and a desired orientation.  dPhi = R - Rd
//
// this and Rd should both be 3x3 rotation matrices.  This operation
// is the following equation, where "X" means cross product:
//    dPhi = -0.5 * ( R[x] X Rd[x]  +  R[y] X Rd[y]  +  R[z] X Rd[z] )
// ===================================================================
void PrMatrix3::angularError( const PrMatrix& Rd, PrVector& dPhi ) const
{
  SAIAssert( Rd.row() == 3 && Rd.column() == 3 );
  dPhi.setSize( 3 );
  const PrMatrix3& R = *this;

  dPhi[0] = (Float) -0.5 * ( R[1][0] * Rd[2][0] - R[2][0] * Rd[1][0] +
                     R[1][1] * Rd[2][1] - R[2][1] * Rd[1][1] +
                     R[1][2] * Rd[2][2] - R[2][2] * Rd[1][2] );
  dPhi[1] = (Float) -0.5 * ( R[2][0] * Rd[0][0] - R[0][0] * Rd[2][0] +
                     R[2][1] * Rd[0][1] - R[0][1] * Rd[2][1] +
                     R[2][2] * Rd[0][2] - R[0][2] * Rd[2][2] );
  dPhi[2] = (Float) -0.5 * ( R[0][0] * Rd[1][0] - R[1][0] * Rd[0][0] +
                     R[0][1] * Rd[1][1] - R[1][1] * Rd[0][1] +
                     R[0][2] * Rd[1][2] - R[1][2] * Rd[0][2] );
}

// ===================================================================
// setRotation(): Create a rotation matrix defined by a rotation axis
// and angle
// ===================================================================
void PrMatrix3::setRotation( const PrVector3& axis, Float angleRadian )
{
  Float c = (Float) cos( angleRadian );
  Float s = (Float) sin( angleRadian );
  Float v = 1-c;

  PrVector3 normAxis( axis );
  normAxis.normalize();
  double x = normAxis[0];
  double y = normAxis[1];
  double z = normAxis[2];

  m_buff[0][0] = (Float) (x*x*v +   c);
  m_buff[0][1] = (Float) (x*y*v - z*s);
  m_buff[0][2] = (Float) (x*z*v + y*s);

  m_buff[1][0] = (Float) (x*y*v + z*s);
  m_buff[1][1] = (Float) (y*y*v +   c);
  m_buff[1][2] = (Float) (y*z*v - x*s);

  m_buff[2][0] = (Float) (x*z*v - y*s);
  m_buff[2][1] = (Float) (y*z*v + x*s);
  m_buff[2][2] = (Float) (z*z*v +   c);
}

// ===================================================================
// setValues(): Same as the base-class method, but optimized for 3x3
// matrices.
// ===================================================================
void PrMatrix3::setValues( const Float* rgVals, bool fTranspose )
{
  if( !fTranspose )
  {
    m_buff[0][0] = rgVals[0];
    m_buff[0][1] = rgVals[1];
    m_buff[0][2] = rgVals[2];
    m_buff[1][0] = rgVals[3];
    m_buff[1][1] = rgVals[4];
    m_buff[1][2] = rgVals[5];
    m_buff[2][0] = rgVals[6];
    m_buff[2][1] = rgVals[7];
    m_buff[2][2] = rgVals[8];
  }
  else
  {
    m_buff[0][0] = rgVals[0];
    m_buff[1][0] = rgVals[1];
    m_buff[2][0] = rgVals[2];
    m_buff[0][1] = rgVals[3];
    m_buff[1][1] = rgVals[4];
    m_buff[2][1] = rgVals[5];
    m_buff[0][2] = rgVals[6];
    m_buff[1][2] = rgVals[7];
    m_buff[2][2] = rgVals[8];
  }
}

// ===================================================================
// zero(): Same as the base-class method, but optimized for 3x3
// matrices.
// ===================================================================
void PrMatrix3::zero()
{
  m_buff[0][0] = 0;
  m_buff[0][1] = 0;
  m_buff[0][2] = 0;
  m_buff[1][0] = 0;
  m_buff[1][1] = 0;
  m_buff[1][2] = 0;
  m_buff[2][0] = 0;
  m_buff[2][1] = 0;
  m_buff[2][2] = 0;
}

// ===================================================================
// identity(): Same as the base-class method, but optimized for 3x3
// matrices.
// ===================================================================
void PrMatrix3::identity()
{
  m_buff[0][0] = 1;
  m_buff[0][1] = 0;
  m_buff[0][2] = 0;

  m_buff[1][0] = 0;
  m_buff[1][1] = 1;
  m_buff[1][2] = 0;

  m_buff[2][0] = 0;
  m_buff[2][1] = 0;
  m_buff[2][2] = 1;
}

// ===================================================================
// negate(): Same as the base-class method, but optimized for 3x3
// matrices.
// ===================================================================
void PrMatrix3::negate( PrMatrix& dest ) const
{
  dest.setSize( 3, 3 );
  dest[0][0] = -m_buff[0][0];
  dest[0][1] = -m_buff[0][1];
  dest[0][2] = -m_buff[0][2];
  dest[1][0] = -m_buff[1][0];
  dest[1][1] = -m_buff[1][1];
  dest[1][2] = -m_buff[1][2];
  dest[2][0] = -m_buff[2][0];
  dest[2][1] = -m_buff[2][1];
  dest[2][2] = -m_buff[2][2];
}

// ===================================================================
// Arithmetic operations: Same as the base-class method, but optimized
// for 3x3 matrices.
// ===================================================================
void PrMatrix3::add( const PrMatrix& rhs, PrMatrix& dest ) const
{
  SAIAssert( rhs.row() == 3 && rhs.column() == 3 );
  dest.setSize( 3, 3 );
  dest[0][0] = m_buff[0][0] + rhs[0][0];
  dest[0][1] = m_buff[0][1] + rhs[0][1];
  dest[0][2] = m_buff[0][2] + rhs[0][2];
  dest[1][0] = m_buff[1][0] + rhs[1][0];
  dest[1][1] = m_buff[1][1] + rhs[1][1];
  dest[1][2] = m_buff[1][2] + rhs[1][2];
  dest[2][0] = m_buff[2][0] + rhs[2][0];
  dest[2][1] = m_buff[2][1] + rhs[2][1];
  dest[2][2] = m_buff[2][2] + rhs[2][2];
}

void PrMatrix3::add( const PrMatrix& rhs )
{
  SAIAssert( rhs.row() == 3 && rhs.column() == 3 );
  m_buff[0][0] += rhs[0][0];
  m_buff[0][1] += rhs[0][1];
  m_buff[0][2] += rhs[0][2];
  m_buff[1][0] += rhs[1][0];
  m_buff[1][1] += rhs[1][1];
  m_buff[1][2] += rhs[1][2];
  m_buff[2][0] += rhs[2][0];
  m_buff[2][1] += rhs[2][1];
  m_buff[2][2] += rhs[2][2];
}

void PrMatrix3::subtract( const PrMatrix& rhs, PrMatrix& dest ) const
{
  SAIAssert( rhs.row() == 3 && rhs.column() == 3 );
  dest.setSize( 3, 3 );
  dest[0][0] = m_buff[0][0] - rhs[0][0];
  dest[0][1] = m_buff[0][1] - rhs[0][1];
  dest[0][2] = m_buff[0][2] - rhs[0][2];
  dest[1][0] = m_buff[1][0] - rhs[1][0];
  dest[1][1] = m_buff[1][1] - rhs[1][1];
  dest[1][2] = m_buff[1][2] - rhs[1][2];
  dest[2][0] = m_buff[2][0] - rhs[2][0];
  dest[2][1] = m_buff[2][1] - rhs[2][1];
  dest[2][2] = m_buff[2][2] - rhs[2][2];
}

void PrMatrix3::subtract( const PrMatrix& rhs )
{
  SAIAssert( rhs.row() == 3 && rhs.column() == 3 );
  m_buff[0][0] -= rhs[0][0];
  m_buff[0][1] -= rhs[0][1];
  m_buff[0][2] -= rhs[0][2];
  m_buff[1][0] -= rhs[1][0];
  m_buff[1][1] -= rhs[1][1];
  m_buff[1][2] -= rhs[1][2];
  m_buff[2][0] -= rhs[2][0];
  m_buff[2][1] -= rhs[2][1];
  m_buff[2][2] -= rhs[2][2];
}

void PrMatrix3::multiply( const PrMatrix& rhs, PrMatrix& dest,
                                               bool fTranspose ) const
{
  if ( rhs.row() != 3 || rhs.column() != 3 )
  {
    // If rhs is not 3x3, then just use the generic parent-class method
    PrMatrix::multiply( rhs, dest, fTranspose );
  }
  else if ( !fTranspose )
  {
    // dest = this * rhs, where rhs is 3x3
    SAIAssert( &dest != static_cast<const PrMatrix*>(this) && &dest != &rhs );
    dest.setSize( 3, 3 );
    const Float (*a)[3] = m_buff;
    const Float (*b)[3] = reinterpret_cast<const Float(*)[3]>( rhs.dataPtr() );
    Float       (*c)[3] = reinterpret_cast<Float(*)[3]>( dest.dataPtr() );

    c[0][0] = a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0];
    c[0][1] = a[0][0] * b[0][1] + a[0][1] * b[1][1] + a[0][2] * b[2][1];
    c[0][2] = a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2] * b[2][2];

    c[1][0] = a[1][0] * b[0][0] + a[1][1] * b[1][0] + a[1][2] * b[2][0];
    c[1][1] = a[1][0] * b[0][1] + a[1][1] * b[1][1] + a[1][2] * b[2][1];
    c[1][2] = a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2] * b[2][2];

    c[2][0] = a[2][0] * b[0][0] + a[2][1] * b[1][0] + a[2][2] * b[2][0];
    c[2][1] = a[2][0] * b[0][1] + a[2][1] * b[1][1] + a[2][2] * b[2][1];
    c[2][2] = a[2][0] * b[0][2] + a[2][1] * b[1][2] + a[2][2] * b[2][2];
  }
  else
  {
    // dest = this * rhs^T, where rhs is 3x3
    SAIAssert( &dest != static_cast<const PrMatrix*>(this) && &dest != &rhs );
    dest.setSize( 3, 3 );
    const Float (*a)[3] = m_buff;
    const Float (*b)[3] = reinterpret_cast<const Float(*)[3]>( rhs.dataPtr() );
    Float       (*c)[3] = reinterpret_cast<Float(*)[3]>( dest.dataPtr() );

    c[0][0] = a[0][0] * b[0][0] + a[0][1] * b[0][1] + a[0][2] * b[0][2];
    c[0][1] = a[0][0] * b[1][0] + a[0][1] * b[1][1] + a[0][2] * b[1][2];
    c[0][2] = a[0][0] * b[2][0] + a[0][1] * b[2][1] + a[0][2] * b[2][2];

    c[1][0] = a[1][0] * b[0][0] + a[1][1] * b[0][1] + a[1][2] * b[0][2];
    c[1][1] = a[1][0] * b[1][0] + a[1][1] * b[1][1] + a[1][2] * b[1][2];
    c[1][2] = a[1][0] * b[2][0] + a[1][1] * b[2][1] + a[1][2] * b[2][2];

    c[2][0] = a[2][0] * b[0][0] + a[2][1] * b[0][1] + a[2][2] * b[0][2];
    c[2][1] = a[2][0] * b[1][0] + a[2][1] * b[1][1] + a[2][2] * b[1][2];
    c[2][2] = a[2][0] * b[2][0] + a[2][1] * b[2][1] + a[2][2] * b[2][2];
  }
}

void PrMatrix3::multiply( const PrVector& rhs, PrVector& dest ) const
{
  SAIAssert( &dest != &rhs );
  SAIAssert( rhs.size() == 3 );
  dest.setSize( 3 );
  const Float (*lhs)[3] = m_buff;

  dest[0] = lhs[0][0] * rhs[0] + lhs[0][1] * rhs[1] + lhs[0][2] * rhs[2];
  dest[1] = lhs[1][0] * rhs[0] + lhs[1][1] * rhs[1] + lhs[1][2] * rhs[2];
  dest[2] = lhs[2][0] * rhs[0] + lhs[2][1] * rhs[1] + lhs[2][2] * rhs[2];
}

void PrMatrix3::multiply( Float rhs, PrMatrix& dest ) const
{
  dest.setSize( 3, 3 );
  dest[0][0] = m_buff[0][0] * rhs;
  dest[0][1] = m_buff[0][1] * rhs;
  dest[0][2] = m_buff[0][2] * rhs;
  dest[1][0] = m_buff[1][0] * rhs;
  dest[1][1] = m_buff[1][1] * rhs;
  dest[1][2] = m_buff[1][2] * rhs;
  dest[2][0] = m_buff[2][0] * rhs;
  dest[2][1] = m_buff[2][1] * rhs;
  dest[2][2] = m_buff[2][2] * rhs;
}

void PrMatrix3::multiply( Float rhs )
{
  m_buff[0][0] *= rhs;
  m_buff[0][1] *= rhs;
  m_buff[0][2] *= rhs;
  m_buff[1][0] *= rhs;
  m_buff[1][1] *= rhs;
  m_buff[1][2] *= rhs;
  m_buff[2][0] *= rhs;
  m_buff[2][1] *= rhs;
  m_buff[2][2] *= rhs;
}

void PrMatrix3::multiplyTranspose( const PrMatrix& rhs, PrMatrix& dest ) const
{
  if ( rhs.row() != 3 || rhs.column() != 3 )
  {
    // If rhs is not 3x3, then just use the generic parent-class method
    PrMatrix::multiplyTranspose( rhs, dest );
  }
  else
  {
    // dest = this^T * rhs, where rhs is 3x3
    SAIAssert( &dest != static_cast<const PrMatrix*>(this) && &dest != &rhs );
    dest.setSize( 3, 3 );
    const PrMatrix3& a = *this;
    const PrMatrix& b = rhs;

    dest[0][0] = a[0][0] * b[0][0] + a[1][0] * b[1][0] + a[2][0] * b[2][0];
    dest[0][1] = a[0][0] * b[0][1] + a[1][0] * b[1][1] + a[2][0] * b[2][1];
    dest[0][2] = a[0][0] * b[0][2] + a[1][0] * b[1][2] + a[2][0] * b[2][2];
    dest[1][0] = a[0][1] * b[0][0] + a[1][1] * b[1][0] + a[2][1] * b[2][0];
    dest[1][1] = a[0][1] * b[0][1] + a[1][1] * b[1][1] + a[2][1] * b[2][1];
    dest[1][2] = a[0][1] * b[0][2] + a[1][1] * b[1][2] + a[2][1] * b[2][2];
    dest[2][0] = a[0][2] * b[0][0] + a[1][2] * b[1][0] + a[2][2] * b[2][0];
    dest[2][1] = a[0][2] * b[0][1] + a[1][2] * b[1][1] + a[2][2] * b[2][1];
    dest[2][2] = a[0][2] * b[0][2] + a[1][2] * b[1][2] + a[2][2] * b[2][2];
  }
}

void PrMatrix3::multiplyTranspose( const PrVector& rhs, PrVector& dest ) const
{
  SAIAssert( &dest != &rhs );
  SAIAssert( rhs.size() == 3 );
  dest.setSize( 3 );
  const PrMatrix3& lhs = *this;

  dest[0] = lhs[0][0] * rhs[0] + lhs[1][0] * rhs[1] + lhs[2][0] * rhs[2];
  dest[1] = lhs[0][1] * rhs[0] + lhs[1][1] * rhs[1] + lhs[2][1] * rhs[2];
  dest[2] = lhs[0][2] * rhs[0] + lhs[1][2] * rhs[1] + lhs[2][2] * rhs[2];
}

void PrMatrix3::transpose( PrMatrix& dest ) const
{
  SAIAssert( &dest != static_cast<const PrMatrix*>(this) );
  dest.setSize( 3, 3 );
  dest[0][0] = m_buff[0][0];
  dest[0][1] = m_buff[1][0];
  dest[0][2] = m_buff[2][0];
  dest[1][0] = m_buff[0][1];
  dest[1][1] = m_buff[1][1];
  dest[1][2] = m_buff[2][1];
  dest[2][0] = m_buff[0][2];
  dest[2][1] = m_buff[1][2];
  dest[2][2] = m_buff[2][2];
}

// ===================================================================
// display(): Display the matrix
// ===================================================================
void PrMatrix3::display( const char* name ) const
{
  if( name != NULL )
  {
    printf( "%s =\n[%.6f %.6f %.6f]\n[%.6f %.6f %.6f]\n[%.6f %.6f %.6f]\n",
            name,
            m_buff[0][0], m_buff[0][1], m_buff[0][2],
            m_buff[1][0], m_buff[1][1], m_buff[1][2],
            m_buff[2][0], m_buff[2][1], m_buff[2][2] );
  }
  else
  {
    printf( "[%.6f %.6f %.6f]\n[%.6f %.6f %.6f]\n[%.6f %.6f %.6f]\n",
            m_buff[0][0], m_buff[0][1], m_buff[0][2],
            m_buff[1][0], m_buff[1][1], m_buff[1][2],
            m_buff[2][0], m_buff[2][1], m_buff[2][2] );
  }
}

// ===================================================================
// dynamic(): This virtual function tells the base that m_data was
// not dynamically allocated in this object.
// ===================================================================
bool PrMatrix3::dynamic() const
{
  return false;
}

// ===================================================================
// resize(): Virtual function that the base class uses to resize the
// matrix.  It is an error to resize a PrMatrix3 to any size other
// than 3x3.
// ===================================================================
void PrMatrix3::resize( int row, int col )
{
  SAIAssert( row == 3 && col == 3 );
}

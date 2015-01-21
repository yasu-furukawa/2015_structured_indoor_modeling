#ifndef NUMERIC_MAT_H
#define NUMERIC_MAT_H

class Cmat {
 public:
  Cmat(void) {
    m_row = 0;
    m_col = 0;
  }
  
  Cmat(const int row, const int col) {
    m_row = row;
    m_col = col;
  }

  virtual ~Cmat() {
  }

  inline int getRow(void) const{
    return m_row;
  }

  inline int getCol(void) const{
    return m_col;
  }
  
  int m_row;
  int m_col;
  
 protected:
};

#endif // MAT_H

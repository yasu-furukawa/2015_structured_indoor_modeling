#ifndef NUMERIC_SPARSEMATALGEBRA_H
#define NUMERIC_SPARSEMATALGEBRA_H

template<class T>
CsparseMat<T>& CsparseMat<T>::operator=(const CsparseMat<T>& rhs) {
  m_row = rhs.m_row;
  m_col = rhs.m_col;
  
  m_val = rhs.m_val;
  m_colind = rhs.m_colind;
  m_rowind = rhs.m_rowind;

  return *this;
};

template <class T>
CsparseMat<T>& CsparseMat<T>::operator+=(const CsparseMat<T>& rhs) {
  const int m = m_row;
  const int n = m_col;
  CsparseMat<T> ans;

  ans.m_row = m;
  ans.m_col = n;
  ans.m_rowind.clear();

  for (int r = 0; r < m; ++r) {
    ans.m_rowind.push_back((int)ans.m_colind.size());
    
    int begin0 = m_rowind[r];
    const int end0 = m_rowind[r+1];
    int begin1 = rhs.m_rowind[r];
    const int end1 = rhs.m_rowind[r+1];

    while (begin0 != end0 || begin1 != end1) {
      if (begin0 == end0) {
	ans.m_colind.insert(ans.m_colind.end(), rhs.m_colind.begin() + begin1,
			    rhs.m_colind.begin() + end1);
	ans.m_val.insert(ans.m_val.end(), rhs.m_val.begin() + begin1,
			 rhs.m_val.begin() + end1);
	break;
      }

      if (begin1 == end1) {
	ans.m_colind.insert(ans.m_colind.end(), m_colind.begin() + begin0,
			    m_colind.begin() + end0);
	ans.m_val.insert(ans.m_val.end(), m_val.begin() + begin0,
			 m_val.begin() + end0);
	break;
      }

      const int col0 = m_colind[begin0];
      const int col1 = rhs.m_colind[begin1];
      if (col0 < col1) {
	ans.m_colind.push_back(col0);
	ans.m_val.push_back(m_val[begin0]);
	begin0++;
      }
      else if (col0 > col1) {
	ans.m_colind.push_back(col1);
	ans.m_val.push_back(rhs.m_val[begin1]);
	begin1++;
      }
      else {
	const T sum = m_val[begin0] + rhs.m_val[begin1];
	if (sum != 0.0) {
	  ans.m_colind.push_back(col0);
	  ans.m_val.push_back(sum);
	}
	begin0++;
	begin1++;
      }
    }
  }
  ans.m_rowind.push_back((int)ans.m_colind.size());

  *this = ans;
  return *this;
};

template <class T>
CsparseMat<T> CsparseMat<T>::operator+(const CsparseMat<T>& rhs) const{
  return CsparseMat<T>(*this) += rhs;
};

template <class T>
CsparseMat<T>& CsparseMat<T>::operator-=(const CsparseMat<T>& rhs) {
  const int m = getRow();
  const int n = getCol();
  CsparseMat<T> ans;
  
  ans.m_row = m;
  ans.m_col = n;
  ans.m_rowind.clear();
  
  for (int r = 0; r < m; ++r) {
    ans.m_rowind.push_back((int)ans.m_colind.size());

    int begin0 = m_rowind[r];
    const int end0 = m_rowind[r+1];
    int begin1 = rhs.m_rowind[r];
    const int end1 = rhs.m_rowind[r+1];

    while (begin0 != end0 || begin1 != end1) {
      if (begin0 == end0) {
	ans.m_colind.insert(ans.m_colind.end(), rhs.m_colind.begin() + begin1,
			    rhs.m_colind.begin() + end1);

	for (int i = begin1; i < end1; ++i)
	  ans.m_val.push_back(-rhs.m_val[i]);
	break;
      }

      if (begin1 == end1) {
	ans.m_colind.insert(ans.m_colind.end(), m_colind.begin() + begin0,
			    m_colind.begin() + end0);
	ans.m_val.insert(ans.m_val.end(), m_val.begin() + begin0,
			 m_val.begin() + end0);
	break;
      }

      const int col0 = m_colind[begin0];
      const int col1 = rhs.m_colind[begin1];
      if (col0 < col1) {
	ans.m_colind.push_back(col0);
	ans.m_val.push_back(m_val[begin0]);
	begin0++;
      }
      else if (col0 > col1) {
	ans.m_colind.push_back(col1);
	ans.m_val.push_back(-rhs.m_val[begin1]);
	begin1++;
      }
      else {
	const T diff = m_val[begin0] - rhs.m_val[begin1];
	if (diff != 0.0) {
	  ans.m_colind.push_back(col0);
	  ans.m_val.push_back(diff);
	}
	begin0++;
	begin1++;
      }
    }
  }

  ans.m_rowind.push_back((int)ans.m_colind.size());

  *this = ans;
  return *this;
};

template <class T>
CsparseMat<T> CsparseMat<T>::operator-(const CsparseMat<T>& rhs) const{
  return CsparseMat<T>(*this) -= rhs;
};

template <class T>
CsparseMat<T>& CsparseMat<T>::operator*=(const T a) {
  for (int i = 0; i < (int)m_val.size(); ++i)
    m_val[i] *= a;
  return *this;
};

template <class T>
CsparseMat<T> CsparseMat<T>::operator*(const T a) const{
  return CsparseMat<T>(*this) *= a;
};

template <class T>
CsparseMat<T>& CsparseMat<T>::operator/=(const T a) {
  for (int i = 0; i < (int)m_val.size(); ++i)
    m_val[i] /= a;
  return *this;
};

template <class T>
CsparseMat<T> CsparseMat<T>::operator/(const T a) const{
  return CsparseMat<T>(*this) /= a;
};

template <class T>
Cvec<T> CsparseMat<T>::operator*(const Cvec<T>& vec) const{
#ifdef FURUKAWA_DEBUG
  if (getCol() != vec.size()) {
    std::cerr << "Invalid matrix vector sizes: " << getRow() << ' ' << getCol()
	      << ' ' << vec.size() << std::endl;
    exit (1);
  }
#endif

  Cvec<T> ans;
  // resize ans vector
  ans.resize(getRow());

  for (int r = 0; r < ans.size(); ++r) {
    ans[r] = 0.0;

    const int beg = m_rowind[r];
    const int end = m_rowind[r+1];
    for (int i = beg; i < end; ++i) {
      const int col = m_colind[i];
      ans[r] += vec[col] * m_val[i];
    }
  }
  return ans;
};

template <class T>
CsparseMat<T>& CsparseMat<T>::operator*=(const CsparseMat<T>& rhs) {
  CsparseMat ans;
  if (getCol() != rhs.getRow()) {
    std::cerr << "Invalid matrix sizes: " << getCol() << ' ' << getRow() << ' '
	      << rhs.getRow() << ' ' << rhs.getCol() << std::endl;    exit (1);
  }
  
  // do not deallocate for efficiency
  ans.m_row = m_row;  ans.m_col = rhs.m_col;
  ans.m_val.clear();
  ans.m_colind.clear();
  ans.m_rowind.clear();
  
  // for each row
  for (int r = 0; r < ans.m_row; ++r) {
    ans.m_rowind.push_back((int)ans.m_colind.size());

    // compute a linear combination of vectors.
    // first collect row indexes of vectors used in rhs and their weights
    std::vector<int> rindexes;    std::vector<T> weights;
    const int beg = m_rowind[r];
    const int end = m_rowind[r+1];
    for (int i = beg; i < end; ++i) {
      rindexes.push_back(m_colind[i]);
      weights.push_back(m_val[i]);
    }

    // next collect final column indexes that will appear in ans
    std::vector<int> cindexes;
    for (int i = 0; i < (int)rindexes.size(); ++i) {
      const int& rtmp = rindexes[i];
      for (int j = rhs.m_rowind[rtmp]; j < rhs.m_rowind[rtmp + 1]; ++j)
	cindexes.push_back(rhs.m_colind[j]);
    }
    // make cindexes unique
    sort(cindexes.begin(), cindexes.end());
    cindexes.erase(unique(cindexes.begin(), cindexes.end()), cindexes.end());
    
    // update ans.m_col
    ans.m_colind.insert(ans.m_colind.end(), cindexes.begin(), cindexes.end());
    
    // finally compute new elements added to ans.m_val for the r_{th}
    // row.  for each element, use a binary search to identify the
    // corresponding entry in ans.m_val, and add its contribution.
    std::vector<T> vals;
    vals.resize((int)cindexes.size());
    for (int i = 0; i < (int)vals.size(); ++i)
      vals[i] = 0.0;

    for (int i = 0; i < (int)rindexes.size(); ++i) {
      const int& rtmp = rindexes[i];
      for (int j = rhs.m_rowind[rtmp]; j < rhs.m_rowind[rtmp + 1]; ++j) {
	const int offset = lower_bound(cindexes.begin(), cindexes.end(),
				       rhs.m_colind[j]) - cindexes.begin();
	vals[offset] += weights[i] * rhs.m_val[j];
      }
    }

    ans.m_val.insert(ans.m_val.end(), vals.begin(), vals.end());
  }
  ans.m_rowind.push_back((int)ans.m_colind.size());

  *this = ans;
  return *this;
};

template <class T>
CsparseMat<T> CsparseMat<T>::operator*(const CsparseMat<T>& rhs) const {
  return CsparseMat<T>(*this) *= rhs;
};

//======================================================================
// linear algebra
//======================================================================
template <class T>
CsparseMat<T> operator*(const T a, const CsparseMat<T>& lhs) {
  return CsparseMat<T>(lhs) *= a;
};

#endif // SPARSEMATALGEBRA_H

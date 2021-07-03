#ifndef CONSTRAINERS_H
#define CONSTRAINERS_H

/*
template<typename TYPE>
inline TYPE& min(TYPE& a, TYPE& b) {(a>b) ? return b : return a;}

template<typename TYPE>
inline TYPE& max(TYPE& a, TYPE& b) {(a<b) ? return b : return a;}

template<typename TYPE>
inline TYPE& constrain(TYPE& input, TYPE& min, TYPE& max) {
    if (input > max) return max;
    if (input < min) return min;
    return input;
}*/



/*template<class A, class B>
constexpr auto min(A&& a, B&& b) -> decltype(a < b ? std::forward<A>(a) : std::forward<B>(b)) {
  return a < b ? std::forward<A>(a) : std::forward<B>(b);
}
template<class A, class B>
constexpr auto max(A&& a, B&& b) -> decltype(a < b ? std::forward<A>(a) : std::forward<B>(b)) {
  return a >= b ? std::forward<A>(a) : std::forward<B>(b);
}*/



#endif
/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

template <typename T>
void split(T data[], unsigned int n, T x, unsigned int& i, unsigned int& j)
{
  do {
    while (data[i] < x) i++;
    while (x < data[j]) j--;

    if (i <= j) {
      T ii = data[i];
      data[i] = data[j];
      data[j] = ii;
      i++; j--;
    }
  } while (i <= j);
}

// C.A.R. Hoare's Quick Median
template <typename T>
unsigned int quick_median(T data[], unsigned int n)
{
  unsigned int l = 0, r = n-1, k = n/2;
  while (l < r) {
    T x = data[k];
    unsigned int i = l, j = r;
    split(data, n, x, i, j);
    if (j < k) l = i;
    if (k < i) r = j;
  }
  return k;
}

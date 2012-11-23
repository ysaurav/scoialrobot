/**
 * @copyright
 *
 * Copyright 2012 Kevin Schluff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2,
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include "hist.h"

using namespace cv;
using namespace std;

inline void calc_hist_bgrd ( Mat& bgr, Mat& depth, Mat& hist )
{
  static const int channels[] = {0, 1, 2, 3};
  static const int b_bins = 8;
  static const int g_bins = 8;
  static const int r_bins = 8;
  static const int d_bins = 8;
  static const int hist_size[] = {b_bins, g_bins, r_bins, d_bins};
  static const float branges[] = {0, 255};
  static const float granges[] = {0, 255};
  static const float rranges[] = {0, 255};
  static const float dranges[] = {0, 255};
  static const float* ranges[] = {branges, granges, rranges, dranges};
  static const Mat mask;
  static const int dims = 4;
  Mat srcs[] = {bgr, depth};

  calcHist ( srcs, sizeof ( srcs ), channels, mask, hist, dims, hist_size, ranges, true, false );
}

inline void calc_hist_d ( Mat& depth, Mat& hist )
{
  static const int channels[] = {0};
  static const int d_bins = 32;
  static const int hist_size[] = {d_bins};
  static const float dranges[] = {0, 255};
  static const float* ranges[] = {dranges};
  static const Mat mask;
  static const int dims = 1;

  Mat srcs[] = {depth};

  calcHist ( srcs, sizeof ( srcs ), channels, mask, hist, dims, hist_size, ranges, true, false );
}

inline void calc_hist_bgr ( Mat& bgr, Mat& hist )
{
  static const int channels[] = {0, 1, 2};
  static const int b_bins = 8;
  static const int g_bins = 8;
  static const int r_bins = 8;
  static const int hist_size[] = {b_bins, g_bins, r_bins};
  static const float branges[] = {0, 255};
  static const float granges[] = {0, 255};
  static const float rranges[] = {0, 255};
  static const float* ranges[] = {branges, granges, rranges};
  static const Mat mask;
  static const int dims = 3;
  Mat srcs[] = {bgr};

  calcHist ( srcs, sizeof ( srcs ), channels, mask, hist, dims, hist_size, ranges, true, false );
}

void calc_hist ( Mat& bgr, Mat& depth, Mat& hist, int type )
{
  switch ( type )
    {
    case 0:
      calc_hist_bgr ( bgr, hist );
      break;
    case 1:
      calc_hist_d ( bgr, hist );
      break;
    case 2:
      calc_hist_bgrd ( bgr, depth, hist );      
      break;
    default:
      break;
    }
}

//
// File: end_point_filter.cpp
//
// Code generated for Simulink model 'end_point_filter'.
//
// Model version                  : 1.142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Sat Jun  8 11:31:02 2019
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "end_point_filter.h"

extern real_T PadBSymm_uD(const real_T u[], const int32_T uStride[], int32_T
  idx[], const int32_T uDims[], const int32_T uStart[], const int32_T uEnd[],
  const int32_T sNumPreEdges, const int32_T sPreEdges[], const int32_T
  sNumPostEdges, const int32_T sPostEdges[]);
extern void MdnFltH15_M_BSymm_uD_yD(const int32_T hLoc[], const int32_T hDims[],
  const real_T u[], const int32_T uDims[], real_T y[], const int32_T yDims[],
  const int32_T yOrigin[]);
real_T PadBSymm_uD(const real_T u[], const int32_T uStride[], int32_T idx[],
                   const int32_T uDims[], const int32_T uStart[], const int32_T
                   uEnd[], const int32_T sNumPreEdges, const int32_T sPreEdges[],
                   const int32_T sNumPostEdges, const int32_T sPostEdges[])
{
  real_T uOut;
  int32_T edgeNum;
  int32_T idxI;

  // S-Function (svipmdnfilter): '<S1>/Median Filter'
  // For boundary sectors:
  // -check if index is outside of input bounds ...
  //   -if so then return pad extended value
  //   -else fetch input data at the index
  // loop through preEdges
  for (edgeNum = 0; edgeNum < sNumPreEdges; edgeNum++) {
    if (idx[sPreEdges[edgeNum]] < uStart[sPreEdges[edgeNum]]) {
      // redirect index across a pre edge
      // reflect across pre-edge
      // check if it is a case of multiple reflections
      // get modulo distance
      idxI = ((uStart[sPreEdges[edgeNum]] - idx[sPreEdges[edgeNum]]) - 1) %
        (uDims[sPreEdges[edgeNum]] << 1);

      // if distance<dim then index is safe to be reflected
      if (idxI < uDims[sPreEdges[edgeNum]]) {
        idxI += uStart[sPreEdges[edgeNum]];

        // else d%dim is reflected across opposite edge
      } else {
        idxI = uEnd[sPreEdges[edgeNum]] - idxI % uDims[sPreEdges[edgeNum]];
      }

      idx[sPreEdges[edgeNum]] = idxI;
    }
  }

  // loop through postEdges
  for (edgeNum = 0; edgeNum < sNumPostEdges; edgeNum++) {
    if (idx[sPostEdges[edgeNum]] > uEnd[sPostEdges[edgeNum]]) {
      // redirect index across a post edge
      // reflect across post-edge
      // check if it is a case of multiple reflections
      // get modulo distance
      idxI = ((idx[sPostEdges[edgeNum]] - uEnd[sPostEdges[edgeNum]]) - 1) %
        (uDims[sPostEdges[edgeNum]] << 1);

      // if distance<dim then index is safe to be reflected
      if (idxI < uDims[sPostEdges[edgeNum]]) {
        idxI = uEnd[sPostEdges[edgeNum]] - idxI;

        // else d%dim is reflected across opposite edge
      } else {
        idxI = idxI % uDims[sPostEdges[edgeNum]] + uStart[sPostEdges[edgeNum]];
      }

      idx[sPostEdges[edgeNum]] = idxI;
    }
  }

  // linearize index and return input data
  uOut = u[idx[0U] * uStride[0U] + idx[1U] * uStride[1U]];

  // End of S-Function (svipmdnfilter): '<S1>/Median Filter'
  return uOut;
}

void MdnFltH15_M_BSymm_uD_yD(const int32_T hLoc[], const int32_T hDims[], const
  real_T u[], const int32_T uDims[], real_T y[], const int32_T yDims[], const
  int32_T yOrigin[])
{
  real_T acc;
  real_T uW[15];
  real_T pivot;
  int32_T uOrigin[2];
  int32_T uEnd[2];
  int32_T uStride[2];
  int32_T hEnd[2];
  int32_T yEnd[2];
  int32_T mLoc[10];
  int32_T mWidth[10];
  int32_T hPre;
  int32_T hPost;
  int32_T bPreEnd;
  int32_T bPostEnd;
  int32_T sStart[2];
  int32_T sEnd[2];
  int32_T sPreEdges[2];
  int32_T sPostEdges[2];
  int32_T c[2];
  int8_T o[5];
  boolean_T isEnd;
  boolean_T isSEmpty;
  boolean_T dimIsPre;
  int32_T offset[2];
  int32_T idxA[2];
  int32_T idxB[2];
  int32_T idxBLin[2];
  int32_T hIdxA[2];
  int32_T hIdxB[2];
  int32_T order;
  int32_T left;
  int32_T right;
  int32_T mid;
  int32_T rightIdx;
  real_T temp;

  // S-Function (svipmdnfilter): '<S1>/Median Filter'
  uOrigin[0U] = 0;
  uEnd[0U] = uDims[0U] - 1;
  uStride[0U] = 1;
  uOrigin[1U] = 0;
  uEnd[1U] = uDims[1U] - 1;
  uStride[1U] = uDims[0U];
  hEnd[0U] = hDims[0U] - 1;
  hEnd[1U] = hDims[1U] - 1;
  yEnd[0U] = yDims[0U] - 1;
  yEnd[1U] = yDims[1U] - 1;

  //  COMPUTE MARGINS BETWEEN INPUT AND OUTPUT
  // ===============================================
  //      _________________________________________
  //     |u                                        |
  //     |                                         |
  //     |         ______________________          |
  //     |        |y                     |         |
  //     |        |                      |         |
  //     |<-yPre->|                      |<-yPost->|
  //     |        |                      |         |
  //
  //  KERNEL MARIGINS
  // ==============================================
  //      ____________________
  //     |h                   |
  //     |        |<--hPost-->|
  //     |        |           |
  //     |        x(hCenter)  |
  //     |<-hPre->|           |
  //     |        |           |
  //
  //  MARGINS (Valid, Boundary and Outside) definitions:
  //  Valid Margin: Processing region where entire kernel stays inside input boundaries. All input data thus considered is 'valid' data 
  //  Boundary Margin: Processing region where part of kernel stays inside the input boundaries and part of it goes outside input boundaries. 
  //  Outside Margin: Processing region where entire kernel stays outside the input boundaries and never touches input data. In most situations this region does not need to be processed. Instead output pad values are filled in 
  // ===========================================================
  //   y |        |        |               |         |         |
  //     |<-oPre->|<-bPre->|<----valid---->|<-bPost->|<-oPost->|
  //     |        |        |               |         |         |
  //
  //  All margin locations are stored relative to the output origin
  // margins between y and u
  right = -yOrigin[0];
  mid = (yOrigin[0] + yEnd[0]) - uEnd[0];

  // margins caused by kernel - defined by center location
  // hLoc and hEnd are defined relative to origin
  hPre = -hLoc[0];
  hPost = hLoc[0] + hEnd[0];

  // margins outside of kernel's influence around input
  // note that we subtract h'Post' from y'Pre'
  rightIdx = right - hPost;

  // note that we subtract h'Pre' from y'Post'
  left = mid - hPre;
  hPre += right;
  hPost += mid;

  // W refers to margin Width
  // L refers to margin Location
  // outer Pre
  // saturate
  if (rightIdx < 0) {
    right = 0;
  } else if (rightIdx > yDims[0]) {
    right = yDims[0];
  } else {
    right = rightIdx;
  }

  // boundary Pre
  // saturate
  if (hPre < 0) {
    hPre = 0;
  } else {
    if (hPre > yDims[0]) {
      hPre = yDims[0];
    }
  }

  if (rightIdx < 0) {
    rightIdx = 0;
  } else {
    if (rightIdx > yDims[0]) {
      rightIdx = yDims[0];
    }
  }

  mid = hPre - rightIdx;
  rightIdx = yOrigin[0] + right;

  // outer Post
  // saturate
  if (left < 0) {
    hPre = 0;
  } else if (left > yDims[0]) {
    hPre = yDims[0];
  } else {
    hPre = left;
  }

  order = ((yOrigin[0] + yEnd[0]) - hPre) + 1;

  // boundary Post
  // saturate
  if (hPost < 0) {
    hPost = 0;
  } else {
    if (hPost > yDims[0]) {
      hPost = yDims[0];
    }
  }

  if (left < 0) {
    left = 0;
  } else {
    if (left > yDims[0]) {
      left = yDims[0];
    }
  }

  left = hPost - left;
  hPost = order - left;
  bPreEnd = rightIdx + mid;
  bPostEnd = hPost + left;
  if (bPreEnd - 1 >= hPost) {
    // defines situation where hDims > uDims
    mid = hPost - rightIdx;
    hPost = bPreEnd;
    left = bPostEnd - bPreEnd;
  }

  // assign to the margin set of mLoc and mWidth
  mLoc[0] = rightIdx + mid;
  mWidth[0] = (((yDims[0] - right) - hPre) - mid) - left;
  mLoc[2] = rightIdx;
  mWidth[2] = mid;
  mLoc[4] = hPost;
  mWidth[4] = left;
  mLoc[6] = yOrigin[0];
  mWidth[6] = right;
  mLoc[8] = order;
  mWidth[8] = hPre;

  // margins between y and u
  right = -yOrigin[1];
  mid = (yOrigin[1] + yEnd[1]) - uEnd[1];

  // margins caused by kernel - defined by center location
  // hLoc and hEnd are defined relative to origin
  hPre = -hLoc[1];
  hPost = hLoc[1] + hEnd[1];

  // margins outside of kernel's influence around input
  // note that we subtract h'Post' from y'Pre'
  rightIdx = right - hPost;

  // note that we subtract h'Pre' from y'Post'
  left = mid - hPre;
  hPre += right;
  hPost += mid;

  // W refers to margin Width
  // L refers to margin Location
  // outer Pre
  // saturate
  if (rightIdx < 0) {
    right = 0;
  } else if (rightIdx > yDims[1]) {
    right = yDims[1];
  } else {
    right = rightIdx;
  }

  // boundary Pre
  // saturate
  if (hPre < 0) {
    hPre = 0;
  } else {
    if (hPre > yDims[1]) {
      hPre = yDims[1];
    }
  }

  if (rightIdx < 0) {
    rightIdx = 0;
  } else {
    if (rightIdx > yDims[1]) {
      rightIdx = yDims[1];
    }
  }

  mid = hPre - rightIdx;
  rightIdx = yOrigin[1] + right;

  // outer Post
  // saturate
  if (left < 0) {
    hPre = 0;
  } else if (left > yDims[1]) {
    hPre = yDims[1];
  } else {
    hPre = left;
  }

  order = ((yOrigin[1] + yEnd[1]) - hPre) + 1;

  // boundary Post
  // saturate
  if (hPost < 0) {
    hPost = 0;
  } else {
    if (hPost > yDims[1]) {
      hPost = yDims[1];
    }
  }

  if (left < 0) {
    left = 0;
  } else {
    if (left > yDims[1]) {
      left = yDims[1];
    }
  }

  left = hPost - left;
  hPost = order - left;
  bPreEnd = rightIdx + mid;
  bPostEnd = hPost + left;
  if (bPreEnd - 1 >= hPost) {
    // defines situation where hDims > uDims
    mid = hPost - rightIdx;
    hPost = bPreEnd;
    left = bPostEnd - bPreEnd;
  }

  // assign to the margin set of mLoc and mWidth
  mLoc[1] = rightIdx + mid;
  mWidth[1] = (((yDims[1] - right) - hPre) - mid) - left;
  mLoc[3] = rightIdx;
  mWidth[3] = mid;
  mLoc[5] = hPost;
  mWidth[5] = left;
  mLoc[7] = yOrigin[1];
  mWidth[7] = right;
  mLoc[9] = order;
  mWidth[9] = hPre;

  //  //////////////////////////////////////////
  //  COMPUTE ON-BOUNDARY SECTORS
  //  //////////////////////////////////////////
  // ===========================================
  //   |   0   |   1   |   2   |   3   |   4   |
  //   | Inside| BndPre|BndPost| OutPre|OutPost|
  //    _______________________________________
  //   |       |///////|///////|       |       |
  //   | VALID |///////|///////|       |       |
  //  0| (0,0) |/(0,1)/|/(0,2)/| (0,3) | (0,4) |
  //   |       |///////|///////|       |       |
  //    _______________________________________
  //   |///////|///////|///////|       |       |
  //  1|/(1,0)/|/(1,1)/|/(1,2)/| (1,3) | (1,4) |
  //   |///////|///////|///////|       |       |
  //    _______________________________________
  //   |///////|///////|///////|       |       |
  //  2|/(2,0)/|/(2,1)/|/(2,2)/| (2,3) | (2,4) |
  //   |///////|///////|///////|       |       |
  //    _______________________________________
  //   |       |       |       |       |       |
  //  3| (3,0) | (3,1) | (3,2) | (3,3) | (3,4) |
  //   |       |       |       |       |       |
  //    _______________________________________
  //   |       |       |       |       |       |
  //  4| (4,0) | (4,1) | (4,2) | (4,3) | (4,4) |
  //   |       |       |       |       |       |
  //    ---------------------------------------
  o[0U] = 0;
  o[1U] = 1;
  o[2U] = 2;
  o[3U] = 3;
  o[4U] = 4;
  isEnd = false;
  c[0U] = 0;
  c[1U] = 0;

  // if hDims < uDims then the all INSIDE sector will not be pushed into the boundary sectors... hence the counter needs to skip the all INSIDEs sector and start from one sector down. 
  while (!isEnd) {
    hPre = 0;
    hPost = 0;

    // compute on-boundary sector
    // compute sector if not empty
    if (!((mWidth[o[c[0U]] << 1] <= 0) || (mWidth[(o[c[1U]] << 1) + 1] <= 0))) {
      // compute sector for this dim
      // sector start corresponds to margin loc
      // apply offset of margin width for sector end
      // Inside Sector if 0 in all dimensions
      isSEmpty = (o[c[0]] == 0);
      dimIsPre = ((o[c[0]] == 3) || (o[c[0]] == 1));
      if (dimIsPre || isSEmpty) {
        sPreEdges[0] = 0;
        hPre = 1;
      }

      if ((!(dimIsPre || isSEmpty)) || isSEmpty) {
        sPostEdges[0] = 0;
        hPost = 1;
      }

      // compute sector for this dim
      // sector start corresponds to margin loc
      // apply offset of margin width for sector end
      // Inside Sector if 0 in all dimensions
      isSEmpty = (o[c[1]] == 0);
      dimIsPre = ((o[c[1]] == 3) || (o[c[1]] == 1));
      if (dimIsPre || isSEmpty) {
        sPreEdges[hPre] = 1;
        hPre++;
      }

      if ((!(dimIsPre || isSEmpty)) || isSEmpty) {
        sPostEdges[hPost] = 1;
        hPost++;
      }

      // setup indices for the loops
      // Origin stores the origin of Y with respect to U
      //  where the origin of Y is at its top left corner
      // Compute offset between origin of Y and origin of U(0,0)
      offset[0U] = -yOrigin[0U];
      offset[1U] = -yOrigin[1U];

      // sector start and end were computed relative to U
      // make them relative to Y by adding offset
      sStart[0U] = mLoc[o[c[0]] << 1] + offset[0U];
      sEnd[0U] = ((mLoc[o[c[0]] << 1] + mWidth[o[c[0]] << 1]) + offset[0U]) - 1;
      sStart[1U] = mLoc[(o[c[1]] << 1) + 1] + offset[1U];
      sEnd[1U] = ((mLoc[(o[c[1]] << 1) + 1] + mWidth[(o[c[1]] << 1) + 1]) +
                  offset[1U]) - 1;

      // adjust offset for kernel center
      offset[0U] -= hLoc[0U];
      offset[1U] -= hLoc[1U];

      // loop kernel over data
      idxB[1U] = sStart[1U];
      while (idxB[1U] <= sEnd[1U]) {
        idxA[1U] = idxB[1U] - offset[1U];
        idxBLin[1U] = yDims[0U] * idxB[1U];
        idxB[0U] = sStart[0U];
        while (idxB[0U] <= sEnd[0U]) {
          idxA[0U] = idxB[0U] - offset[0U];
          idxBLin[0U] = idxBLin[1U] + idxB[0U];

          // loop over kernel and compute conv/corr
          bPreEnd = 0;

          // loop over kernel
          hIdxA[1U] = 0;
          while (hIdxA[1U] <= hEnd[1U]) {
            hIdxB[1U] = idxA[1U] + hIdxA[1U];
            hIdxA[0U] = 0;
            while (hIdxA[0U] <= hEnd[0U]) {
              hIdxB[0U] = idxA[0U] + hIdxA[0U];
              temp = PadBSymm_uD(&u[0U], &uStride[0U], &hIdxB[0U], &uDims[0U],
                                 &uOrigin[0U], &uEnd[0U], hPre, &sPreEdges[0U],
                                 hPost, &sPostEdges[0U]);

              // cache away u for the window
              uW[bPreEnd] = temp;
              bPreEnd++;
              hIdxA[0U]++;
            }

            hIdxA[1U]++;
          }

          bPreEnd = hDims[0U] * hDims[1U] - 1;
          order = ((bPreEnd + 2) >> 1) - 1;
          if ((bPreEnd + 1) % 2 == 0) {
            // Perform the scanning only if the current partition has
            // at least three elements.
            left = 0;
            right = bPreEnd;
            while (right > left + 1) {
              // Use the median of the three values as the partitioning key
              // Use median-of-three method to return the partitioning key (pivot). 
              // This method makes worst-case behavior much less likely and
              // avoids the needs for sentinel values outside the array.
              // //////////////////////////////////////////////////////////////////// 
              // First we sort the first, middle, and last element of the partition. 
              mid = (left + right) >> 1;
              if (uW[left] > uW[mid]) {
                temp = uW[left];
                uW[left] = uW[mid];
                uW[mid] = temp;
              }

              if (uW[left] > uW[right]) {
                temp = uW[left];
                uW[left] = uW[right];
                uW[right] = temp;
              }

              if (uW[mid] > uW[right]) {
                temp = uW[mid];
                uW[mid] = uW[right];
                uW[right] = temp;
              }

              // //////////////////////////////////////////////////////////////////// 
              // Now swap the middle value with the next-to-last value.
              temp = uW[mid];
              uW[mid] = uW[right - 1];
              uW[right - 1] = temp;

              // return pivot for partitioning
              pivot = uW[right - 1];

              // Start the partitioning scan.  Note that because we sorted
              // the left and right values, we can start the comparisons
              // with (left+1) and (right-2).  This is how we avoid the
              // need for sentinel values.
              mid = left;
              rightIdx = right - 1;
              isSEmpty = true;
              while (isSEmpty) {
                do {
                  mid++;
                } while (uW[mid] < pivot);

                do {
                  rightIdx--;
                } while (uW[rightIdx] > pivot);

                // pointers crossed -> end scan
                if (mid >= rightIdx) {
                  isSEmpty = false;
                } else {
                  // Swap values at end of current interval.
                  temp = uW[mid];
                  uW[mid] = uW[rightIdx];
                  uW[rightIdx] = temp;
                }
              }

              // One last swap needed at end of scan
              temp = uW[mid];
              uW[mid] = uW[right - 1];
              uW[right - 1] = temp;

              // Select the left or right subpartition depending on
              // the value of order.
              if (mid >= order) {
                right = mid - 1;
              }

              if (mid <= order) {
                left = mid + 1;
              }
            }

            if ((right - left == 1) && (uW[left] > uW[right])) {
              // Last partition has two elements that may not be sorted.
              temp = uW[left];
              uW[left] = uW[right];
              uW[right] = temp;
            }

            acc = uW[order];

            // Perform the scanning only if the current partition has
            // at least three elements.
            left = 0;
            right = bPreEnd;
            while (right > left + 1) {
              // Use the median of the three values as the partitioning key
              // Use median-of-three method to return the partitioning key (pivot). 
              // This method makes worst-case behavior much less likely and
              // avoids the needs for sentinel values outside the array.
              // //////////////////////////////////////////////////////////////////// 
              // First we sort the first, middle, and last element of the partition. 
              mid = (left + right) >> 1;
              if (uW[left] > uW[mid]) {
                temp = uW[left];
                uW[left] = uW[mid];
                uW[mid] = temp;
              }

              if (uW[left] > uW[right]) {
                temp = uW[left];
                uW[left] = uW[right];
                uW[right] = temp;
              }

              if (uW[mid] > uW[right]) {
                temp = uW[mid];
                uW[mid] = uW[right];
                uW[right] = temp;
              }

              // //////////////////////////////////////////////////////////////////// 
              // Now swap the middle value with the next-to-last value.
              temp = uW[mid];
              uW[mid] = uW[right - 1];
              uW[right - 1] = temp;

              // return pivot for partitioning
              pivot = uW[right - 1];

              // Start the partitioning scan.  Note that because we sorted
              // the left and right values, we can start the comparisons
              // with (left+1) and (right-2).  This is how we avoid the
              // need for sentinel values.
              mid = left;
              rightIdx = right - 1;
              isSEmpty = true;
              while (isSEmpty) {
                do {
                  mid++;
                } while (uW[mid] < pivot);

                do {
                  rightIdx--;
                } while (uW[rightIdx] > pivot);

                // pointers crossed -> end scan
                if (mid >= rightIdx) {
                  isSEmpty = false;
                } else {
                  // Swap values at end of current interval.
                  temp = uW[mid];
                  uW[mid] = uW[rightIdx];
                  uW[rightIdx] = temp;
                }
              }

              // One last swap needed at end of scan
              temp = uW[mid];
              uW[mid] = uW[right - 1];
              uW[right - 1] = temp;

              // Select the left or right subpartition depending on
              // the value of order.
              if (mid >= order + 1) {
                right = mid - 1;
              }

              if (mid <= order + 1) {
                left = mid + 1;
              }
            }

            if ((right - left == 1) && (uW[left] > uW[right])) {
              // Last partition has two elements that may not be sorted.
              temp = uW[left];
              uW[left] = uW[right];
              uW[right] = temp;
            }

            acc += uW[order + 1];
            acc /= 2.0;
          } else {
            // Perform the scanning only if the current partition has
            // at least three elements.
            left = 0;
            right = bPreEnd;
            while (right > left + 1) {
              // Use the median of the three values as the partitioning key
              // Use median-of-three method to return the partitioning key (pivot). 
              // This method makes worst-case behavior much less likely and
              // avoids the needs for sentinel values outside the array.
              // //////////////////////////////////////////////////////////////////// 
              // First we sort the first, middle, and last element of the partition. 
              mid = (left + right) >> 1;
              if (uW[left] > uW[mid]) {
                temp = uW[left];
                uW[left] = uW[mid];
                uW[mid] = temp;
              }

              if (uW[left] > uW[right]) {
                temp = uW[left];
                uW[left] = uW[right];
                uW[right] = temp;
              }

              if (uW[mid] > uW[right]) {
                temp = uW[mid];
                uW[mid] = uW[right];
                uW[right] = temp;
              }

              // //////////////////////////////////////////////////////////////////// 
              // Now swap the middle value with the next-to-last value.
              temp = uW[mid];
              uW[mid] = uW[right - 1];
              uW[right - 1] = temp;

              // return pivot for partitioning
              pivot = uW[right - 1];

              // Start the partitioning scan.  Note that because we sorted
              // the left and right values, we can start the comparisons
              // with (left+1) and (right-2).  This is how we avoid the
              // need for sentinel values.
              mid = left;
              rightIdx = right - 1;
              isSEmpty = true;
              while (isSEmpty) {
                do {
                  mid++;
                } while (uW[mid] < pivot);

                do {
                  rightIdx--;
                } while (uW[rightIdx] > pivot);

                // pointers crossed -> end scan
                if (mid >= rightIdx) {
                  isSEmpty = false;
                } else {
                  // Swap values at end of current interval.
                  temp = uW[mid];
                  uW[mid] = uW[rightIdx];
                  uW[rightIdx] = temp;
                }
              }

              // One last swap needed at end of scan
              temp = uW[mid];
              uW[mid] = uW[right - 1];
              uW[right - 1] = temp;

              // Select the left or right subpartition depending on
              // the value of order.
              if (mid >= order) {
                right = mid - 1;
              }

              if (mid <= order) {
                left = mid + 1;
              }
            }

            if ((right - left == 1) && (uW[left] > uW[right])) {
              // Last partition has two elements that may not be sorted.
              temp = uW[left];
              uW[left] = uW[right];
              uW[right] = temp;
            }

            acc = uW[order];
          }

          if (idxBLin[0U] >= 0) {
            y[idxBLin[0U]] = acc;
          }

          idxB[0U]++;
        }

        idxB[1U]++;
      }
    }

    // advance sector counter
    hPre = 0;
    while (hPre < 2) {
      c[hPre]++;
      if (c[hPre] <= 2) {
        isEnd = false;
        hPre = 2;
      } else {
        c[hPre] = 0;
        isEnd = true;
      }

      hPre++;
    }
  }

  // End of S-Function (svipmdnfilter): '<S1>/Median Filter'
}

// Model step function
void end_poing_filter::step(real_T (&arg_end_point)[3], real_T
  (&arg_end_point_filtered)[3])
{
  int32_T h1Dims[2];
  int32_T uDims[2];
  int32_T yDims[2];
  int32_T hLoc[2];
  int32_T yOrigin[2];
  real_T rateLimiterRate;
  int32_T h1Dims_0[2];
  int32_T uDims_0[2];
  int32_T yDims_0[2];
  int32_T hLoc_0[2];
  int32_T yOrigin_0[2];
  int32_T h1Dims_1[2];
  int32_T uDims_1[2];
  int32_T yDims_1[2];
  int32_T hLoc_1[2];
  int32_T yOrigin_1[2];
  real_T rtb_RateLimiter2;
  real_T rtb_RateLimiter1;
  real_T rtb_RateLimiter;

  // S-Function (svipmdnfilter): '<S1>/Median Filter' incorporates:
  //   Inport: '<Root>/end_point'

  h1Dims[0U] = 5;
  h1Dims[1U] = 3;
  uDims[0U] = 1;
  uDims[1U] = 1;

  // set up kernel related coordinates
  // compute center
  // hLoc is the location of top left corner relative to the center of kernel.
  hLoc[0U] = -2;

  // compute center
  // hLoc is the location of top left corner relative to the center of kernel.
  hLoc[1U] = -1;

  // Region of Support (ROS) definition: A selected region that restricts the input space for processing. 
  // ======================================================
  //                       ---------------------
  //                      |ROI                  |
  //                      |                     |
  //  ----------------------------------------------------
  // | INPUT(u)           |                     |         |
  // |                    |                     |         |
  // |   ---------------------------------------------    |
  // |  | ROS             |OUTPUT(y)////////////|     |   |
  // |  |                 |/////////////////////|     |   |
  // |  |                  ---------------------      |   |
  // |  |                                             |   |
  // The user's output mode choices of 'Valid', 'Same as input' and 'Full' map onto correspending definitions of a rectangular ROS. 
  // Output range support is computed as an intersection of ROS with Region of Interest (ROI) 
  // /////////////////////
  // begin ROS computation
  // compute ROS based on u, h and output mode
  // ROS is SAME AS INPUT
  // end ROS computation
  // ///////////////////
  // /////////////////////////
  // begin y sizes computation
  // we need to compute yOrigin and yEnd
  // yOrigin definition: Location of center of reference (origin) of output (y) coordintate system with respect to input (u) coordinate system 
  // y sizes are same as ros sizes because there is no ROI
  yOrigin[0U] = 0;
  yDims[0U] = 1;

  // y sizes are same as ros sizes because there is no ROI
  yOrigin[1U] = 0;
  yDims[1U] = 1;

  // make yOrigin same as roiLoc when y is empty
  // end y sizes computation
  // ///////////////////////
  MdnFltH15_M_BSymm_uD_yD(&hLoc[0U], &h1Dims[0U], &arg_end_point[0], &uDims[0U],
    &rtb_RateLimiter, &yDims[0U], &yOrigin[0U]);

  // RateLimiter: '<S1>/Rate Limiter'
  rateLimiterRate = rtb_RateLimiter - end_point_filter_DW.PrevY;
  if (rateLimiterRate > 0.0008) {
    rtb_RateLimiter = end_point_filter_DW.PrevY + 0.0008;
  } else {
    if (rateLimiterRate < -0.0008) {
      rtb_RateLimiter = end_point_filter_DW.PrevY + -0.0008;
    }
  }

  end_point_filter_DW.PrevY = rtb_RateLimiter;

  // End of RateLimiter: '<S1>/Rate Limiter'

  // S-Function (svipmdnfilter): '<S1>/Median Filter1' incorporates:
  //   Inport: '<Root>/end_point'

  h1Dims_0[0U] = 5;
  h1Dims_0[1U] = 3;
  uDims_0[0U] = 1;
  uDims_0[1U] = 1;

  // set up kernel related coordinates
  // compute center
  // hLoc is the location of top left corner relative to the center of kernel.
  hLoc_0[0U] = -2;

  // compute center
  // hLoc is the location of top left corner relative to the center of kernel.
  hLoc_0[1U] = -1;

  // Region of Support (ROS) definition: A selected region that restricts the input space for processing. 
  // ======================================================
  //                       ---------------------
  //                      |ROI                  |
  //                      |                     |
  //  ----------------------------------------------------
  // | INPUT(u)           |                     |         |
  // |                    |                     |         |
  // |   ---------------------------------------------    |
  // |  | ROS             |OUTPUT(y)////////////|     |   |
  // |  |                 |/////////////////////|     |   |
  // |  |                  ---------------------      |   |
  // |  |                                             |   |
  // The user's output mode choices of 'Valid', 'Same as input' and 'Full' map onto correspending definitions of a rectangular ROS. 
  // Output range support is computed as an intersection of ROS with Region of Interest (ROI) 
  // /////////////////////
  // begin ROS computation
  // compute ROS based on u, h and output mode
  // ROS is SAME AS INPUT
  // end ROS computation
  // ///////////////////
  // /////////////////////////
  // begin y sizes computation
  // we need to compute yOrigin and yEnd
  // yOrigin definition: Location of center of reference (origin) of output (y) coordintate system with respect to input (u) coordinate system 
  // y sizes are same as ros sizes because there is no ROI
  yOrigin_0[0U] = 0;
  yDims_0[0U] = 1;

  // y sizes are same as ros sizes because there is no ROI
  yOrigin_0[1U] = 0;
  yDims_0[1U] = 1;

  // make yOrigin same as roiLoc when y is empty
  // end y sizes computation
  // ///////////////////////
  MdnFltH15_M_BSymm_uD_yD(&hLoc_0[0U], &h1Dims_0[0U], &arg_end_point[1],
    &uDims_0[0U], &rtb_RateLimiter1, &yDims_0[0U], &yOrigin_0[0U]);

  // RateLimiter: '<S1>/Rate Limiter1'
  rateLimiterRate = rtb_RateLimiter1 - end_point_filter_DW.PrevY_p;
  if (rateLimiterRate > 0.0008) {
    rtb_RateLimiter1 = end_point_filter_DW.PrevY_p + 0.0008;
  } else {
    if (rateLimiterRate < -0.0008) {
      rtb_RateLimiter1 = end_point_filter_DW.PrevY_p + -0.0008;
    }
  }

  end_point_filter_DW.PrevY_p = rtb_RateLimiter1;

  // End of RateLimiter: '<S1>/Rate Limiter1'

  // S-Function (svipmdnfilter): '<S1>/Median Filter2' incorporates:
  //   Inport: '<Root>/end_point'

  h1Dims_1[0U] = 5;
  h1Dims_1[1U] = 3;
  uDims_1[0U] = 1;
  uDims_1[1U] = 1;

  // set up kernel related coordinates
  // compute center
  // hLoc is the location of top left corner relative to the center of kernel.
  hLoc_1[0U] = -2;

  // compute center
  // hLoc is the location of top left corner relative to the center of kernel.
  hLoc_1[1U] = -1;

  // Region of Support (ROS) definition: A selected region that restricts the input space for processing. 
  // ======================================================
  //                       ---------------------
  //                      |ROI                  |
  //                      |                     |
  //  ----------------------------------------------------
  // | INPUT(u)           |                     |         |
  // |                    |                     |         |
  // |   ---------------------------------------------    |
  // |  | ROS             |OUTPUT(y)////////////|     |   |
  // |  |                 |/////////////////////|     |   |
  // |  |                  ---------------------      |   |
  // |  |                                             |   |
  // The user's output mode choices of 'Valid', 'Same as input' and 'Full' map onto correspending definitions of a rectangular ROS. 
  // Output range support is computed as an intersection of ROS with Region of Interest (ROI) 
  // /////////////////////
  // begin ROS computation
  // compute ROS based on u, h and output mode
  // ROS is SAME AS INPUT
  // end ROS computation
  // ///////////////////
  // /////////////////////////
  // begin y sizes computation
  // we need to compute yOrigin and yEnd
  // yOrigin definition: Location of center of reference (origin) of output (y) coordintate system with respect to input (u) coordinate system 
  // y sizes are same as ros sizes because there is no ROI
  yOrigin_1[0U] = 0;
  yDims_1[0U] = 1;

  // y sizes are same as ros sizes because there is no ROI
  yOrigin_1[1U] = 0;
  yDims_1[1U] = 1;

  // make yOrigin same as roiLoc when y is empty
  // end y sizes computation
  // ///////////////////////
  MdnFltH15_M_BSymm_uD_yD(&hLoc_1[0U], &h1Dims_1[0U], &arg_end_point[2],
    &uDims_1[0U], &rtb_RateLimiter2, &yDims_1[0U], &yOrigin_1[0U]);

  // RateLimiter: '<S1>/Rate Limiter2'
  rateLimiterRate = rtb_RateLimiter2 - end_point_filter_DW.PrevY_o;
  if (rateLimiterRate > 0.0008) {
    rtb_RateLimiter2 = end_point_filter_DW.PrevY_o + 0.0008;
  } else {
    if (rateLimiterRate < -0.0008) {
      rtb_RateLimiter2 = end_point_filter_DW.PrevY_o + -0.0008;
    }
  }

  end_point_filter_DW.PrevY_o = rtb_RateLimiter2;

  // End of RateLimiter: '<S1>/Rate Limiter2'

  // Outport: '<Root>/end_point_filtered'
  arg_end_point_filtered[0] = rtb_RateLimiter;
  arg_end_point_filtered[1] = rtb_RateLimiter1;
  arg_end_point_filtered[2] = rtb_RateLimiter2;
}

// Model initialize function
void end_poing_filter::initialize()
{
  // Registration code

  // initialize error status
  rtmSetErrorStatus((&end_point_filter_M), (NULL));

  // states (dwork)
  (void) memset((void *)&end_point_filter_DW, 0,
                sizeof(DW_end_point_filter_T));

  // InitializeConditions for RateLimiter: '<S1>/Rate Limiter'
  end_point_filter_DW.PrevY = 0.0;

  // InitializeConditions for RateLimiter: '<S1>/Rate Limiter1'
  end_point_filter_DW.PrevY_p = 0.0;

  // InitializeConditions for RateLimiter: '<S1>/Rate Limiter2'
  end_point_filter_DW.PrevY_o = 0.0;
}

// Model terminate function
void end_poing_filter::terminate()
{
  // (no terminate code required)
}

// Constructor
end_poing_filter::end_poing_filter()
{
}

// Destructor
end_poing_filter::~end_poing_filter()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
RT_MODEL_end_point_filter_T * end_poing_filter::getRTM()
{
  return (&end_point_filter_M);
}

//
// File trailer for generated code.
//
// [EOF]
//

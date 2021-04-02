# Sipmle BA

## Download the data at here 
- http://grail.cs.washington.edu/projects/bal/

## Residual Implementation 
  - For the loss, follow this equation (i.e., reprojection measurement model) 
    ```
    P  =  R * X + t       (conversion from world to camera coordinates)
    p  = -P / P.z         (perspective division)
    p' =  f * r(p) * p    (conversion to pixel coordinates)
    , where r(p) = 1.0 + k1 * ||p||^2 + k2 * ||p||^4.
    ```
  - Then a difference between the predicted p' = (u', v') and the measured p = (u, v) would be minimized.
    - for the implementation of above lines, see the Residual.h   

## Verification
- using CloudCompare, see the results before-and-after (files indata directory) 

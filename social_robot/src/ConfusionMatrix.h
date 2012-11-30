#ifndef CONFUSIONMATRIX_H
#define CONFUSIONMATRIX_H

/** 
* @class ConfusionMatrix
*
* @brief This class provides confusion matrix (true positves, true negatives, false positves, false negatives)
* for the evaluation between ground-truth and results.
* 
* @author Social Robot
* 
*/

class ConfusionMatrix
{

public:
    
    int tp;
    int tn;
    int fn;
    int fp;
    
    ConfusionMatrix();
  
};

#endif // CONFUSIONMATRIX_H

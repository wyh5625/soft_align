#ifndef NEW_TYPE_H
#define NEW_TYPE_H

enum alignment_type
{
    CORNER,  // Align the corners of the fabric
    KEYPOINT // Align the key points of the fabric
};

enum alignment_state
{
    INIT,     // Initial flattened state
    OALIGNED, // The first key point or corner point (aka. center of rotation) has been aligned
    ROTATED,  // The whole fabric has been rotated about the center of rotation
    CALIGNED, // All corners have been aligned.
    KALIGNED  // All key points have been aligned.
};


#endif

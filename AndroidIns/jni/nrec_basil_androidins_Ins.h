/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
/* Header for class nrec_basil_androidins_Ins */

#ifndef _Included_nrec_basil_androidins_Ins
#define _Included_nrec_basil_androidins_Ins
#ifdef __cplusplus
extern "C" {
#endif
/*
 * Class:     nrec_basil_androidins_Ins
 * Method:    init
 * Signature: (Landroid/content/res/AssetManager;)Z
 */
JNIEXPORT jboolean JNICALL Java_nrec_basil_androidins_Ins_init
  (JNIEnv *, jobject, jobject);

/*
 * Class:     nrec_basil_androidins_Ins
 * Method:    close
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_nrec_basil_androidins_Ins_close
  (JNIEnv *, jobject);

/*
 * Class:     nrec_basil_androidins_Ins
 * Method:    integrate
 * Signature: ([D)V
 */
JNIEXPORT void JNICALL Java_nrec_basil_androidins_Ins_integrate
  (JNIEnv *, jobject, jdoubleArray);

/*
 * Class:     nrec_basil_androidins_Ins
 * Method:    get_pose
 * Signature: ()[D
 */
JNIEXPORT jdoubleArray JNICALL Java_nrec_basil_androidins_Ins_get_1pose
  (JNIEnv *, jobject);

#ifdef __cplusplus
}
#endif
#endif

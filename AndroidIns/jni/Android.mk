APP_STL := stlport_static

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_C_INCLUDES := basil/BASIL-2.3/source \
					/usr/local/include/eigen3 \
					/usr/local/include/boost-1.43 \

LOCAL_MODULE := AndroidIns

LOCAL_SRC_FILES := AndroidIns.cpp

LOCAL_LDLIBS += -landroid \
				-llog

include $(BUILD_SHARED_LIBRARY)
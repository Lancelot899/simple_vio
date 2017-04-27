#include <gtest/gtest.h>

#include "glog/logging.h"

/// Run all the tests that were declared with TEST()
#define TEST_INITIAL

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
#ifdef TEST_ALL
  return RUN_ALL_TESTS();

#else
  #ifdef TEST_INITIAL
  #include "vio/test/Test_initial.h"
		testInitial(argc, argv);
  #else
    #ifdef TEST_BA
	#include "vio/BA/Implement/test/Test_SimpleBA.h"
      testBA(argc, argv);
	#else

    #endif // TEST_BA
  #endif //TEST_INITIAL

#endif //TEST_ALL

  }


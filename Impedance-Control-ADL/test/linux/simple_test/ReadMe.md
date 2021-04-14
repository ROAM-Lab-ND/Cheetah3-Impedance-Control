When run SOEM with rt_eathercat on ADL, make sure GNU is using c99 standard. Add the following to the CMakeList.txt
set(CMAKE_C_FLAGS, "-std=gnu99")

Otherwise, you will have the error such as "for loop initial declaration is not allowed". 





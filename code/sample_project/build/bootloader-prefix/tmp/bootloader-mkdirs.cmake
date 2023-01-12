# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/Hanne/esp/esp-idf/components/bootloader/subproject"
  "C:/Users/Hanne/Documents/school/school2022_2023/Enigma/code/vscodeESPIDF/espv5/sample_project/build/bootloader"
  "C:/Users/Hanne/Documents/school/school2022_2023/Enigma/code/vscodeESPIDF/espv5/sample_project/build/bootloader-prefix"
  "C:/Users/Hanne/Documents/school/school2022_2023/Enigma/code/vscodeESPIDF/espv5/sample_project/build/bootloader-prefix/tmp"
  "C:/Users/Hanne/Documents/school/school2022_2023/Enigma/code/vscodeESPIDF/espv5/sample_project/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/Hanne/Documents/school/school2022_2023/Enigma/code/vscodeESPIDF/espv5/sample_project/build/bootloader-prefix/src"
  "C:/Users/Hanne/Documents/school/school2022_2023/Enigma/code/vscodeESPIDF/espv5/sample_project/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/Hanne/Documents/school/school2022_2023/Enigma/code/vscodeESPIDF/espv5/sample_project/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/Hanne/Documents/school/school2022_2023/Enigma/code/vscodeESPIDF/espv5/sample_project/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()

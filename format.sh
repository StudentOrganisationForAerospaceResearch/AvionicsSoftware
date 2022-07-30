clang-format -i $(find Core/Inc/ Core/Src/ -type f -not -iregex "Core/\(Src\|Inc\)/\(stm32\|syscalls\|sysmem\|system_stm32\|freertos\).*")

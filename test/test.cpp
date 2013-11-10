#include <string>
#include <iostream>
extern "C" {
  void runalgo() {
    std::string haystack("ABACAB is it everywhere!");
    std::string needle1 ("ACAB");
    std::string needle2 ("not ABA");
  }
} // extern "C"

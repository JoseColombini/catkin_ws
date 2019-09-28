#include <fstream>
#include <iostream>
#include <stdio.h>      /* printf, fgets */
#include <stdlib.h>

using namespace std;

int main () {

  ifstream File;
  double vetor[2];
  File.open("Go_Mail_box.txt", std::ios_base::in);
  if(File.is_open()) {
    for(int i = 0; i < 2; i++) {
       File >> vetor[i];
    }
  }else {
    cout<< "No file" << endl;
  }
  cout << vetor[0] << " " << vetor[1];
  return 0;
}

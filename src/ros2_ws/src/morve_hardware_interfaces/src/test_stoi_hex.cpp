#include <format>
#include <string>
#include <iostream>

using namespace std;

int main(int argc, char* argv[]){
    string str_hex_prefix_number = "0x2a";
    string str_hex_number = "2a";
    int number = 42;

    if(argc != 2){
        cerr << "Program requires one argument number." << endl;
        return 1;
    }

    cout << stoi(str_hex_prefix_number, 0, 16) << " " << stoi(str_hex_number, 0, 16) << endl;
    
    cout << std::format("cislo prevedene z int decimal na hexadecimalni cislo 0x{:x} \n", number);
    cout << std::format("Manualne zadane cislo: 0x{:x}", stoi(argv[1]), 0, 16);
    return 0;
}
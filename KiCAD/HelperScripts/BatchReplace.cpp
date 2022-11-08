#include <string>
#include <iostream>
#include <sstream>
#include <fstream>

using namespace std;

string parseQuoteTextFromString(string s)
{
    string ret;
    for (size_t i = 0; i < s.length(); i++)
    {
        if(s.at(i) == '\"')
        {
            while (s.at(++i) != '\"') ret += s.at(i);
        }
    }
    return ret;
}

void parseNumsFromString(string s, float *args, int len)
{
    stringstream strStrm(s);
    // Convert the string s into stringstream.
    // strStrm << s;
    float tmpF;
    int i = 0;
    while (!strStrm.eof())
    {
        string tmpStr = "";
        // Separate words into the temp string one at a time.
        strStrm >> tmpStr;
        if (stringstream(tmpStr) >> tmpF)
        { // try to convert the string to int
            args[i] = tmpF;
            // Don't allow more than four numbers.
            if (++i == len) break;
        }
    }
}

int main(void)
{
    int subSymbol = 1;
    int pinCount = 0;
    string s;

    ifstream ifile("IC,STM32WB55VGQ6.kicad_sym");
    // Use a while loop together with the getline() function to read the file line by line.

    while (getline(ifile, s))
    {
        if (s.find("(symbol") != string::npos)
        {
            subSymbol++;
        }
        if (subSymbol == 0) continue;
        if (s.find("(pin") != string::npos)
        {
            getline(ifile, s);
            if (s.find("(name") == string::npos) continue;
            string name = parseQuoteTextFromString(s);
            float nameSize[2];
            parseNumsFromString(s, nameSize, 2);

            getline(ifile, s);
            if (s.find("(number") == string::npos) continue;
            string number = parseQuoteTextFromString(s);
            float numberSize[2];
            parseNumsFromString(s, numberSize, 2);

            pinCount++;

            cout << "found pin - number: " << pinCount << ", name: " << name << ", name size: " << nameSize[0] \
            << ", " << nameSize[1] << "number: " << number << ", number size: " << numberSize[0] << ", " << numberSize[1] << endl;
        }
    }
}
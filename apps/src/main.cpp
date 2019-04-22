#include "AsyncSerial.h"

#include <iostream>
#include <termios.h>
#include <sstream>

using namespace std;

void received(const char* data, unsigned int len)
{
    vector<char> v(data,data+len);
    for(unsigned int i=0;i<v.size();i++)
    {
        if(v[i]=='\n')
        {
            cout<<endl;
        } else {
            if(v[i]<32 || v[i]>=0x7f) cout.put(' ');//Remove non-ascii char
            else cout.put(v[i]);
        }
    }
    cout.flush();//Flush screen buffer
}

int main(int argc, char* argv[])
{
    #ifdef BOOST_LITTLE_ENDIAN
    cout<< "little" << endl;
    #else
    cout<< "big" << endl;
    #endif
    // if(argc!=3)
    // {
    //     cerr<<"Usage: serial port baudrate"<<endl<<
    //             "To quit type Ctrl-C x"<<endl<<
    //             "To send Ctrl-C type Ctrl-C Ctrl-C"<<endl;
    //     return 1;
    // }

    char line[4] = {(char)0x10, (char)0x01, (char)0xAA, (char)0x01};
    printf( "The size of the array is %zu"
            "\nand the length of the stored string \n%s is %zu\n",
            sizeof( line ), line, strlen( line ) );
    stringstream s;
    cout<<line<<endl;
    s << line;
    uint32_t a;
    s >> a;
    cout<<a << endl;


    termios stored_settings;
    tcgetattr(0, &stored_settings);
    termios new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_lflag &= (~ISIG); // don't automatically handle control-C
    new_settings.c_lflag &= ~(ECHO); // no echo
    tcsetattr(0, TCSANOW, &new_settings);

    cout<<"\e[2J\e[1;1H"; //Clear screen and put cursor to 1;1

    try {
        CallbackAsyncSerial serial(argv[1],stoi(argv[2]));
        serial.setCallback(received);
        for(;;)
        {
            if(serial.errorStatus() || serial.isOpen()==false)
            {
                cerr<<"Error: serial port unexpectedly closed"<<endl;
                break;
            }
            char c;
            cin.get(c); //blocking wait for standard input
            if(c==3) //if Ctrl-C
            {
                cin.get(c);
                switch(c)
                {
                    case 3:
                        serial.write(&c,1);//Ctrl-C + Ctrl-C, send Ctrl-C
                    break;
                    case 'x': //fall-through
                    case 'X':
                        goto quit;//Ctrl-C + x, quit
                    default:
                        serial.write(&c,1);//Ctrl-C + any other char, ignore
                }
            } else serial.write(&c,1);
        }
        quit:
        serial.close();
    } catch (std::exception& e) {
        cerr<<"Exception: "<<e.what()<<endl;
    }

    tcsetattr(0, TCSANOW, &stored_settings);
}

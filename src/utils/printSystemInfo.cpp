/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */


#include <iostream>
#include <climits>
#include <sys/times.h>
#include <unistd.h>

using namespace std;

int main(int argc, char **argv)
{
    cout << "------sizes-----"<<endl;
    cout << "size of a double is "<<sizeof(double)<< " chars."<<endl;
    cout << "One char is " << CHAR_BIT << " bits."<<endl;
    cout << endl;

    cout << "------timing-----"<<endl;
    clock_t ticks_per_sec = sysconf(_SC_CLK_TCK);
    cout << "Ticks per second is "<< ticks_per_sec<<endl;
    cout << endl;
}

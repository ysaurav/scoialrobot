#include "string_utils.h"

#include <sstream>
#include <time.h>

using namespace std;

string inttostr ( int x )
{
  stringstream ss;
  ss << x;
  return ss.str();
}

string floattostr ( double x )
{
    ostringstream ss;
    ss << x;
    return ss.str();
}

string current_log_time ( void )
{
  time_t now;
  time ( &now );

  char buffer [80];
  strftime ( buffer, 20, "%Y%m%d%H%M%S", localtime ( &now ) );
  string log_time = buffer;
  cout << log_time << endl;

  int i;
  int length = log_time.length();
  for ( i = 0; i < length; i++ )
    {
      char c = log_time[i];
      if ( c < '0' || c > '9' )
        {
          //log_time.erase(i, 1);
        }
    }

  return log_time;
}
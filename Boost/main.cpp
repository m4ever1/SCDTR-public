//ASYNC_CONSOLE
#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <unistd.h> //for syscall dup()
#include <thread>
#include <sstream>

using ec = const boost::system::error_code;
using sz = std::size_t;
using boost::asio::chrono::seconds;
using namespace boost::system;
using namespace boost::asio;
//GLOBALS
boost::asio::io_context io;
boost::asio::serial_port sp {io};
boost::asio::deadline_timer tim {io};
boost::asio::streambuf read_buf; //read buffer
boost::asio::streambuf write_buf; //write buffer
int counter = 0;

//HANDLERS FOR ASYNC CALLBACKS
//forward declaration of write_handler to timer_handler
void write_handler(const error_code &ec, size_t nbytes);
//timer_handler
void timer_handler(const error_code &ec)
{
//timer expired – launch new write operation
  std::ostringstream os;
  // os << "Counter = " << ++counter;
}
void write_handler(const error_code &ec, size_t nbytes) {
//writer done – program new deadline
  tim.expires_from_now(boost::posix_time::seconds(5));
  tim.async_wait(timer_handler);

}
void read_handler(const error_code &ec, size_t nbytes) {
//data is now available at read_buf
  std::cout << &read_buf;
//program new read cycle
  async_read_until(sp,read_buf,'\n',read_handler);
}

void writeard()
{
  char arg1;
  int arg2, arg3;
  char superdata[9];
  std::string data;
  int a =1;
  while(a)
  {
    std::getline(std::cin, data);
    
    std::cout << data << std::endl;  
    
    if((data[0] == 'g')||(data[0] == 'o'))
    {
      data[1] = data[2];
      data[2] = data[4];
      if(data[5] != '\0')
      {
        data[3] = data[5];
        data[4] ='\0';
      }
      else
        data[3] = '\0';
    }
    else
    {
      data[1] = data[2];
      if(data[3] == ' ')
      {
        data[2] = ' ';
        data[3] = data[4];
        data[4] = data[5];
        data[5] = data[6];
      }
      else
      {
        data[2] = data[3];
        data[3] = ' ';
        data[4] = data[5];
        data[5] = data[6];
        data[6] = data[7];
      }
    }
    
    std::cout << "Sent " << data << std::endl;
    async_write(sp, boost::asio::buffer(data, 6), write_handler);
  }

}

int main (int argc, char* argv[])
{

  boost::system::error_code ec;
  sp.open("/dev/ttyACM2", ec); //connect to port
  if( ec )
     std::cout << "Could not open serial port" << std::endl;
   sp.set_option(serial_port_base::baud_rate(115200),ec);
//program timer for write operations
  tim.expires_from_now(boost::posix_time::seconds(5));
  tim.async_wait(timer_handler);
//program chain of read operations
  std::thread th1 (writeard);
  async_read_until(sp,read_buf,'\n',read_handler);
  std::string s( (std::istreambuf_iterator<char>(&read_buf)), std::istreambuf_iterator<char>() );
  //output_decoder(s)

while(1)
{
  io.run(); //get things rolling
}
 
}
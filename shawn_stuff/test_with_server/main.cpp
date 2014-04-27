#include <cstring>
#include <iostream>
#include <string>

#include "Planner.h"

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>


using boost::asio::ip::tcp;

Planner p;

class ui_connection
  : public boost::enable_shared_from_this<ui_connection>
{
public:
  typedef boost::shared_ptr<ui_connection> pointer;

  static pointer create(boost::asio::io_service& io_service)
  {
    // std::cout << "created new tcp connection"<<std::endl;
    return pointer(new ui_connection(io_service));
  }

  tcp::socket& socket()
  {
    return socket_;
  }

  void start()
  {
    std::cout << "TCP start\n";
    boost::asio::async_read_until(socket_, data_, "\n",
        boost::bind(&ui_connection::handle_request_line, this, _1));
  }

  void write_message(std::string message)
  {
    message_ = message;
    boost::asio::async_write(socket_, boost::asio::buffer(message_),
        boost::bind(&ui_connection::handle_write, shared_from_this(),
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
  }

  void handle_request_line(boost::system::error_code ec)
  {
    std::cout << "Got new line?\n";
    if (!ec)
    {
      std::istream is(&data_);

      std::stringstream ss;
      ss << is.rdbuf();
      std::string line = ss.str();

      std::cout << "got: "<<line<<"\n";
      if (line.substr(0,7).compare("getpath") == 0)
      {
        double x,y,ex,ey;
        char command[20];
        sscanf(line.c_str(), "%s %lf %lf %lf %lf", command, &x, &y, &ex, &ey);
        //received a get path request... return it!!!
        std::cout << "update the start & end coords: "<<x<< " "<<y<<" "<<ex<<" "<<ey<<"\n";
        p.updateStartEnd(x,y,ex,ey);
        p.runPlanner(5);
        std::vector< std::pair<double, double> > latlong = p.getgps();
        std::stringstream ss;
        for (std::vector< std::pair<double, double> >::iterator i = latlong.begin(); i != latlong.end(); ++i)
        {
            ss<< std::fixed << std::setprecision(std::numeric_limits< double >::digits10) << i->first<<" "<<i->second<<"\n";
        }
        ss << "points end\n";
        write_message(ss.str());
        p.writeSolution("sol_picked.txt");
      }
    }
    else
    {
      throw boost::system::system_error(ec); // Some other error.
    }
  }

  boost::asio::streambuf data_;

private:
  ui_connection(boost::asio::io_service& io_service)
    : socket_(io_service)
  {
  }

  void handle_write(const boost::system::error_code& /*error*/,
      size_t /*bytes_transferred*/)
  {
  }

  std::vector<std::string> data_lines;

  tcp::socket socket_;
  std::string message_;
};

class tcp_server
{
public:
  tcp_server(boost::asio::io_service& io_service, int port)
    : acceptor_(io_service, tcp::endpoint(tcp::v4(), port))
  {
    start_accept();
  }

private:
  void start_accept()
  {
    ui_connection::pointer connection =
      ui_connection::create(acceptor_.io_service());
    new_connections.push_back(connection);

    acceptor_.async_accept(connection->socket(),
        boost::bind(&tcp_server::handle_accept, this, connection,
          boost::asio::placeholders::error));
    // std::cout << "setup new server?"<<std::endl;
  }

  void handle_accept(ui_connection::pointer connection,
      const boost::system::error_code& error)
  {
    if (!error)
    {
      connection->start();
      start_accept();
      // std::cout << "started accept"<<std::endl;
    }
  }
  std::vector<ui_connection::pointer> new_connections;
  tcp::acceptor acceptor_;
};

int main(int argc, char *argv[])
{
    if (argc != 7)
    {
        std::cout << "You need at least 6 arguments: usage =\n"<<argv[0] <<" "<<"<.env config file> <motion primitives file> <start x> <start y> <end x> <end y> \n";
        exit(1);
    }
    double startX = atof(argv[3]);
    double startY = atof(argv[4]);
    double endX = atof(argv[5]);
    double endY = atof(argv[6]);
    std::pair<double, double> start(startX, startY);
    std::pair<double, double> end(endX, endY);
    p.planxythetalat(argv[1], argv[2], start, end);
    std::cout << "Finished planning\n";
    std::vector< std::pair< double, double > > UTM = p.getpath();
    std::cout << "Got path\n";
    std::vector< std::pair< double, double > > GPS = p.convertToGPS(UTM);
    std::cout << "Converted to GPS\n";

    // for (std::vector< std::pair<double,double> >::iterator i = GPS.begin(); i != GPS.end(); ++i)
    // {
        // std::cout << "Lat = "<<(*i).first<<"   long = "<<(*i).second<<std::endl;
    // }

    try
    {
        boost::asio::io_service io_service;
        tcp_server server(io_service, 9090);
        io_service.run();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }

}

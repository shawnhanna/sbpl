//
// server.cpp
// ~~~~~~~~~~
//
// Copyright (c) 2003-2011 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <ctime>
#include <iostream>
#include <string>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

std::string get_path(double startX, double startY, double endX, double endY)
{
  /*
  40.444985 -80.022676
  40.442356 -80.018728
  40.444038 -80.014694
  40.440788 -80.014715
  40.439531 -80.012076
  */

  //get the start coordinate
  std::cout << "startx: "<<startX<<" starty: "<<startY<<" endx: "<<endX<<" endy: "<<endY<<"\n";
  return std::string("40.444985 -80.022676\n40.442356 -80.018728\n40.444038 -80.014694\n40.440788 -80.014715\n40.439531 -80.012076\npoints end");
}

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
    // std::cout << "TCP start?\n";
    boost::asio::async_read_until(socket_, data_, "\r\n",
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
        write_message(get_path(x,y,ex,ey));
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

int main(int argc, char* argv[])
{
  if (argc != 2)
  {
    std::cout << "ERROR: must use format ./waypoint_server <port>"<<std::endl;
    return 1;
  }
  try
  {
    boost::asio::io_service io_service;
    tcp_server server(io_service, atoi(argv[1]));
    io_service.run();
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}

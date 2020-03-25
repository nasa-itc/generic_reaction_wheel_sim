/* Copyright (C) 2016 - 2020 National Aeronautics and Space Administration. All Foreign Rights are Reserved to the U.S. Government.

   This software is provided "as is" without any warranty of any, kind either express, implied, or statutory, including, but not
   limited to, any warranty that the software will conform to, specifications any implied warranties of merchantability, fitness
   for a particular purpose, and freedom from infringement, and any warranty that the documentation will conform to the program, or
   any warranty that the software will be error free.

   In no event shall NASA be liable for any damages, including, but not limited to direct, indirect, special or consequential damages,
   arising out of, resulting from, or in any way connected with the software or its documentation.  Whether or not based upon warranty,
   contract, tort or otherwise, and whether or not loss was sustained from, or arose out of the results of, or use of, the software,
   documentation or services provided hereunder

   ITC Team
   NASA IV&V
   ivv-itc@lists.nasa.gov
*/

#include <generic_rw_hardware_model.hpp>
#include <generic_rw_sim_data_42socket_provider.hpp>

#include <ItcLogger/Logger.hpp>

#include <boost/property_tree/xml_parser.hpp>

namespace Nos3
{
    REGISTER_HARDWARE_MODEL(GenericRWHardwareModel,"GENERICREACTIONWHEELHARDWARE");

    extern ItcLogger::Logger *sim_logger;

    GenericRWHardwareModel::GenericRWHardwareModel(const boost::property_tree::ptree& config) : SimIHardwareModel(config), _keep_running(true)
    {
        sim_logger->trace("GenericRWHardwareModel::GenericRWHardwareModel:  Constructor executing");

        // Here's how to write out the config data passed
        //std::ostringstream oss;
        //write_xml(oss, config);
        //sim_logger->info("GenericRWHardwareModel::GenericRWHardwareModel:  "
        //    "configuration:\n%s", oss.str().c_str());

        // Here's how to get a time node to get time from
        std::string connection_string = config.get("common.nos-connection-string", "tcp://127.0.0.1:12001");

        std::string time_bus_name = "command";
        if (config.get_child_optional("hardware-model.connections")) 
        {
            BOOST_FOREACH(const boost::property_tree::ptree::value_type &v, config.get_child("hardware-model.connections")) 
            {
                if (v.second.get("type", "").compare("time") == 0) 
                {
                    time_bus_name = v.second.get("bus-name", "command");
                    break;
                }
            }
        }
        _time_bus.reset(new NosEngine::Client::Bus(_hub, connection_string, time_bus_name));

        // Here's how to get a UART node to communicate with
        std::string bus_name = "usart_0";
        int node_port = 0;
        if (config.get_child_optional("simulator.hardware-model.connections")) 
        {
            BOOST_FOREACH(const boost::property_tree::ptree::value_type &v, config.get_child("simulator.hardware-model.connections")) 
            {
                if (v.second.get("type", "").compare("usart") == 0) 
                {
                    bus_name = v.second.get("bus-name", bus_name);
                    node_port = v.second.get("node-port", node_port);
                    break;
                }
            }
        }
        _uart_connection.reset(new NosEngine::Uart::Uart(_hub, config.get("simulator.name", "generic-rw-sim"), connection_string,
            bus_name));
        _uart_connection->open(node_port);
        _uart_connection->set_read_callback(
            std::bind(&GenericRWHardwareModel::uart_read_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Here's how to get a data provider
        std::string dp_name = config.get("simulator.hardware-model.data-provider.type", "GENERICRWSIMDATA42SOCKETPROVIDER");
        _sdp = SimDataProviderFactory::Instance().Create(dp_name, config);

        _prev_data_sent_time = _absolute_start_time + 10.0;
        _time_bus->add_time_tick_callback(std::bind(&GenericRWHardwareModel::send_periodic_data, this, std::placeholders::_1));

        sim_logger->trace("GenericRWHardwareModel::GenericRWHardwareModel:  Time node, UART node, data provider created; constructor exiting");
    }

    GenericRWHardwareModel::~GenericRWHardwareModel(void)
    {
        sim_logger->trace("GenericRWHardwareModel::GenericRWHardwareModel:  Destructor executing");
        delete _sdp; // Clean up the data provider we got
        _time_bus.reset(); // Must reset the time bus so the unique pointer does not try to delete the hub.  Do not destroy the time node, the bus will do it
        _uart_connection->close();
    }

    void GenericRWHardwareModel::run(void)
    {
        int i = 0;
        boost::shared_ptr<SimIDataPoint> dp;
        while(_keep_running) 
        {
            sim_logger->trace("GenericRWHardwareModel::run:  Loop count %d, time %f", i++,
                _absolute_start_time + (double(_time_bus->get_time() * _sim_microseconds_per_tick)) / 1000000.0);
            sleep(5);
        }
    }

    void GenericRWHardwareModel::uart_read_callback(const uint8_t *buf, size_t len)
    {
        // Get the data out of the message bytes
        std::vector<uint8_t> in_data(buf, buf + len);
        sim_logger->debug("GenericRWHardwareModel::uart_read_callback:  REQUEST %s",
            SimIHardwareModel::uint8_vector_to_hex_string(in_data).c_str()); // log data in a man readable format

        // Figure out how to respond (possibly based on the in_data)
        std::vector<uint8_t> out_data = in_data; // Just echo

        // Ship the message bytes off
        sim_logger->debug("GenericRWHardwareModel::uart_read_callback:  REPLY   %s\n",
            SimIHardwareModel::uint8_vector_to_hex_string(out_data).c_str()); // log data in a man readable format

        _uart_connection->write(&out_data[0], out_data.size());
    }

    void GenericRWHardwareModel::command_callback(NosEngine::Common::Message msg)
    {
        // Here's how to get the data out of the message
        NosEngine::Common::DataBufferOverlay dbf(const_cast<NosEngine::Utility::Buffer&>(msg.buffer));
        sim_logger->info("GenericRWHardwareModel::command_callback:  Received command: %s.", dbf.data);

        // Do something with the data
        std::string command = dbf.data;
        std::string response = "GenericRWHardwareModel::command_callback:  INVALID COMMAND! (Try STOP FOOSIM)";
        boost::to_upper(command);
        if (command.substr(0,11).compare("STOP RWSIM") == 0) 
        {
            _keep_running = false;
            response = "GenericRWHardwareModel::command_callback:  STOPPING FOOSIM";
        } else if (command.substr(0,6).compare("TORQUE") == 0)
        {
            dynamic_cast<GenericRWData42SocketProvider*>(_sdp)->send_command_to_socket("SC[0].AC.Whl[0].Tcmd = 0.1");
            response = "GenericRWHardwareModel::command_callback:  TORQUING REACTION WHEEL";
        }

        // Here's how to send a reply
        _command_node->send_reply_message_async(msg, response.size(), response.c_str());
    }

    void GenericRWHardwareModel::send_periodic_data(NosEngine::Common::SimTime time)
    {
        const boost::shared_ptr<GenericRWDataPoint> data_point =
            boost::dynamic_pointer_cast<GenericRWDataPoint>(_sdp->get_data_point());

        std::vector<uint8_t> data;

        double abs_time = _absolute_start_time + (double(time * _sim_microseconds_per_tick)) / 1000000.0;
        double next_time = _prev_data_sent_time + _period - (_sim_microseconds_per_tick / 1000000.0) / 2; // within half a tick time period
        if (next_time < abs_time) { // Time to send more data
            _prev_data_sent_time = abs_time;
            create_rw_data(*data_point, data);
            _uart_connection->write(&data[0], data.size());
        }
    }

    typedef union {
        double  d_mom;
        uint8_t u_mom[8];
    } momentum_union;

    void GenericRWHardwareModel::create_rw_data(const GenericRWDataPoint& data_point, std::vector<uint8_t>& out_data)
    {
        momentum_union mu;
        mu.d_mom = data_point.get_momentum();
        out_data.push_back(mu.u_mom[0]);
        out_data.push_back(mu.u_mom[1]);
        out_data.push_back(mu.u_mom[2]);
        out_data.push_back(mu.u_mom[3]);
        out_data.push_back(mu.u_mom[4]);
        out_data.push_back(mu.u_mom[5]);
        out_data.push_back(mu.u_mom[6]);
        out_data.push_back(mu.u_mom[7]);
        sim_logger->debug("GenericRWHardwareModel::create_rw_data:  Momentum:  double=%f, uint8_t[8]=0x%2.2x 0x%2.2x 0x%2.2x 0x%2.2x 0x%2.2x 0x%2.2x 0x%2.2x 0x%2.2x", 
            mu.d_mom, mu.u_mom[0], mu.u_mom[1], mu.u_mom[2], mu.u_mom[3], mu.u_mom[4], mu.u_mom[5], mu.u_mom[6], mu.u_mom[7]);
    }

}

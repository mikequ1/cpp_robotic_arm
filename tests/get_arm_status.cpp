// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>
#include <vector>

#include <franka/exception.h>
#include <franka/robot.h>

 #include "rapidjson/document.h"
 #include "rapidjson/writer.h"
 #include "rapidjson/stringbuffer.h"

#include "examples_common.h"

using namespace std;

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    std::cout << "This example will obtain the current NE_T_EE and O_T_EE positions " << endl
              << "Press Enter to continue..." << endl;
    std::cin.ignore();

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    franka::RobotState init_state = robot.readOnce();
    std::array<double, 16> init_NE_T_EE = init_state.NE_T_EE;
    std::array<double, 16> init_O_T_EE = init_state.O_T_EE;
    std::array<double, 7> init_q = init_state.q;
    cout << "NE_T_EE" << endl;
    for (int i = 0; i < 16; i++) {
      cout << init_NE_T_EE[i] << " ";
    }
    cout << endl << endl;

    rapidjson::Document document;
    document.SetObject();
    rapidjson::Value dataArray(rapidjson::kArrayType);
    rapidjson::Document::AllocatorType& allocator = document.GetAllocator();
    cout << "O_T_EE" << endl;
    for (int i = 0; i < 16; i++) {
      dataArray.PushBack(init_O_T_EE[i], allocator);
      // cout << init_O_T_EE[i] << " ";
    }
    document.AddMember("pose", dataArray, allocator); 
    rapidjson::StringBuffer strbuf;
    rapidjson::Writer<rapidjson::StringBuffer> writer(strbuf);
    document.Accept(writer);

    cout << strbuf.GetString() << endl << endl;

    cout << "Q (joint position)" << endl;
    for (int i = 0; i < 7; i++) {
      cout << init_q[i] << " ";
    }
    cout << endl << endl;

  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}

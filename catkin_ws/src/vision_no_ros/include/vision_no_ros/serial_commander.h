#ifndef SERIAL_COMMANDER_H
#define SERIAL_COMMANDER_H

static int command;

void set_command(){
    std::cout << "enter new command"<< std::endl;
    std::cin >> command;
}



int get_command(){
    return command;
}
#endif

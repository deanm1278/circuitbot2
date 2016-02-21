/* 
 * File:   gcParser.cpp
 * Author: deanmiller
 * 
 * Created on November 27, 2015, 9:38 AM
 */

#include <cctype>
#include <algorithm>
#include <vector>
#include <boost/regex.hpp>
#include <map>
#include <iterator> // for std::begin, std::end

#include "gcParser.h"

const char gcParser::paramLetters[23] = {'A', 'B', 'C', 'D', 'E', 'H', 'I', 'J', 'K', 'L', 'P', 'Q', 'R', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 'F'}; //parameter letters

gcParser::gcParser(){
    //--- command priority defined here --//
    order['G'][93] = 0;   //set feed rate mode
    order['G'][94] = 0;
    order['M'][62] = 4;   //HAL pin I/0
    order['M'][63] = 4;
    order['M'][64] = 4;
    order['M'][65] = 4;
    order['M'][66] = 4;
    order['M'][06] = 5;   //Change tool (M6) and set tool number(M61)
    order['M'][61] = 5;
    order['M'][03] = 6;   //spindle on or off
    order['M'][04] = 6;
    order['M'][05] = 6;
    order['M'][70] = 7;   //slave state, restore state, individual state
    order['M'][73] = 7;
    order['M'][72] = 7;
    order['M'][71] = 7;
    order['M'][7] = 8;   //coolant on or off
    order['M'][8] = 8;
    order['M'][9] = 8;
    order['M'][48] = 9;   //enable or disable overrides
    order['M'][49] = 9;
    order['M'][50] = 9;
    order['M'][51] = 9;
    order['M'][52] = 9;
    order['M'][53] = 9;
    //user defined commands go here
    
    order['G'][04] = 11;   //dwell
    order['G'][17] = 12;  //set active plane
    order['G'][18] = 12;
    order['G'][19] = 12;
    order['G'][20] = 13;  //set length units
    order['G'][21] = 13;
    order['G'][40] = 14;  //cutter radius comp on or off
    order['G'][41] = 14;
    order['G'][42] = 14;
    order['G'][43] = 15;  //cutter length compensation on or off
    order['G'][49] = 15;
    order['G'][54] = 16;  //coordinate system selection
    order['G'][55] = 16;
    order['G'][56] = 16;
    order['G'][57] = 16;
    order['G'][58] = 16;
    order['G'][59] = 16;
    order['G'][61] = 17;  //set path control
    order['G'][64] = 17;
    order['G'][90] = 18;  //set distance mode
    order['G'][91] = 18;
    order['G'][98] = 19;  //set retract mode
    order['G'][99] = 19;
    order['G'][28] = 20;  //ref location, change coordinate sys data, set axis offset
    order['G'][30] = 20;
    order['G'][10] = 20;
    order['G'][92] = 20;
    order['G'][94] = 20;
    order['G'][0] = 21;   //perform motion
    order['G'][3] = 21;
    order['G'][33] = 21;
    order['G'][38] = 21;
    order['G'][73] = 21;
    order['G'][76] = 21;
    order['G'][80] = 21;
    order['G'][89] = 21;
    order['G'][53] = 21;
    order['M'][0] = 22;   //stop
    order['M'][1] = 22;
    order['M'][2] = 22;
    order['M'][30] = 22;
    order['M'][60] = 22;
}

bool gcParser::parseBlock(std::string block, std::vector<cmd_t> &cmds){
    
    static const boost::regex re_fullCmd("("                                    //for parsing the full gcode command
        "([GM]([0-9]{1,3})([ABCDEFHIJKLPQRUVWXYZ](\\+|\\-|\\.|[0-9])+)*)|"
        "([ABCDEFHIJKLPQRSUVWXYZFT]((\\+|\\-|\\.|[0-9])+)*)+"
        ")"),
        re_cmdNumber("[0-9]{1,3}"),                                             //for parsing the command number
        re_params("[ABCDEFHIJKLPQRSTUVWXYZF](\\+|\\-|\\.|[0-9])+");               //for parsing command parameters
    
    int i = strcspn (block.c_str(),";");                                   //remove comments
    block = block.substr(0,i);
    
    block.erase(std::remove_if(block.begin(),block.end(), ::isspace),block.end()); //remove whitespace
    
    boost::sregex_token_iterator iter(block.begin(), block.end(), re_fullCmd, 0);
    boost::sregex_token_iterator end;
    
    for( ; iter != end; ++iter ) {
        cmd_t a;
        a.letter = NO_LETTER;
        a.number = strtof("", NULL);
        
        std::string s = *iter;
        char letter = s.at(0); //get the command letter
        
        bool is_param = std::find(paramLetters, paramLetters + NUM_LETTERS, letter) != paramLetters + 23;
        
        if(!is_param){ //If this is a new command get the letter and number
            if(letter == 'G' || letter == 'M'){
                a.letter = letter;
                boost::smatch result;
                if (boost::regex_search(s, result, re_cmdNumber)) {
                    std::string submatch(result[0]);
                    a.number = strtof(submatch.c_str(), NULL);
                }
            }
        }
        boost::sregex_token_iterator paramIter(s.begin(), s.end(), re_params, 0);
        boost::sregex_token_iterator paramIterEnd;
        for( ; paramIter != paramIterEnd; ++paramIter ) {
            std::string p = *paramIter;
            char p_letter = p.at(0);
            a.params[p_letter] = strtof(p.substr(1, std::string::npos).c_str(), NULL);
        }
        a.priority = order[a.letter][a.number];
        cmds.push_back(a);
    }
    
    std::sort(cmds.begin(), cmds.end()); //sort according to priority
    
    if(!cmds.empty()){
        return 1;
    }
    else{
        return 0;
    }
}


gcParser::~gcParser() {
}


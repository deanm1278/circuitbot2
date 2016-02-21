/* 
 * File:   gcParser.h
 * Author: deanmiller
 *
 * Created on November 27, 2015, 9:38 AM
 */

#include <vector>
#include <map>

#ifndef GCPARSER_H
#define	GCPARSER_H

#define NO_LETTER '\0'
#define NUM_LETTERS 23

typedef struct cmd_t{
    char letter;
    float number;
    int priority;
    std::map<char, float> params;
    
    bool operator < (const cmd_t& str) const
    {
        return (priority < str.priority);
    }
} cmd_t;

class gcParser {
public:
    gcParser();
    bool parseBlock(std::string block, std::vector<cmd_t> &cmds);
    
    static const char paramLetters[NUM_LETTERS];
    static const char paramCmds[3];
    
    virtual ~gcParser();
private:
    
    std::map <char, std::map<float, int> > order;

};

#endif	/* GCPARSER_H */


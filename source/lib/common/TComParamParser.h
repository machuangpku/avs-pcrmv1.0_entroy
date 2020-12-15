#pragma once
/* The copyright in this software is being made available under the BSD
* License, included below. This software may be subject to other third party
* and contributor rights, including patent rights, and no such rights are
* granted under this license.
*
* Copyright (c) 2019-2033, Audio Video coding Standard Workgroup of China
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*  * Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*  * Neither the name of Audio Video coding Standard Workgroup of China
*    nor the names of its contributors maybe used to endorse or promote products
*    derived from this software without
*    specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "contributors.h"
#include <algorithm>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string.h>
#include <vector>

using namespace std;

///< \in TLibCommon \{

enum KeyType {
  KT_INVALID = 0,  ///< invalid key
  KT_SHORT = 1,    ///< single dash parameter
  KT_LONG = 2,     ///< double dash parameter
  KT_FILE = 3,     ///< parse from configure file
};

///< function pointer of configure file parser
typedef void(ConfigParserFunc)(const string&, vector<string>&, vector<string>&);

/**
 * struct ParameterBase
 * basic parameter structure
 */

struct ParameterBase {
  string nameShort;
  string nameLong;
  string prompt;

  ParameterBase() {}
  ParameterBase(const string nameShort, const string nameLong, const string prompt)
    : nameShort(nameShort)
    , nameLong(nameLong)
    , prompt(prompt) {}

  virtual void init() = 0;
  virtual void set(const string& str) = 0;
  virtual void printHelp(ostream& out, int maxLengthLong, int maxLengthShort) = 0;
  virtual void printValue(ostream& out, int maxLengthLong) = 0;
};  ///< END STRUCT ParameterBase

/**
 * struct ConfigParameter : ParameterBase
 * config file parameter structure
 */

struct ConfigParameter : ParameterBase {
  ConfigParserFunc* configParser;

  ConfigParameter() {}
  ConfigParameter(ConfigParserFunc* parser, const string nameShort, const string nameLong,
                  const string prompt)
    : configParser(parser) {
    this->nameShort = nameShort;
    this->nameLong = nameLong;
    this->prompt = prompt;
  }

  void init() {}
  void set(const string& str) {}
  void printHelp(ostream& out, int maxLengthLong, int maxLengthShort) {}
  void printValue(ostream& out, int maxLengthLong) {}

};  ///< END STRUCT ConfigParameter

/**
 * struct Parameter : ParameterBase
 * generic parameter structure
 */

template<typename T> struct Parameter : ParameterBase {
  T& data;
  T defVal;

  Parameter(T& data, const T defVal, const string nameShort, const string nameLong,
            const string prompt)
    : data(data)
    , defVal(defVal) {
    this->nameShort = nameShort;
    this->nameLong = nameLong;
    this->prompt = prompt;
  }

  void set(const string& str) {
    stringstream ss(str, istringstream::in);
    try {
      ss >> data;
    } catch (...) {
      cerr << "Error: fail to read parameter --" << nameLong << endl;
    }
  }

  void init() {
    data = defVal;
  }

  void printHelp(ostream& out, int maxLengthLong, int maxLengthShort) {
    out << left << setw(maxLengthShort + 2) << "-" + nameShort;
    out << left << setw(maxLengthLong + 3) << "--" + nameLong;
    out << prompt << endl;
  }

  void printValue(ostream& out, int maxLengthLong) {
    out << left << setw(maxLengthLong + 3) << "--" + nameLong;
    out << "= " << data << endl;
  }
};  ///< END STRUCT Parameter

typedef vector<ParameterBase*> Parameters;

/**
 * struct ParameterGroup
 * parameter group
 */

struct ParameterGroup {
  string groupName;
  Parameters params;

  ParameterGroup(const string& name) {
    groupName = name;
  }

  void printGroupName(ostream& out) {
    if (groupName.length() != 0) {
      out << endl << "<" << groupName << ">" << endl;
    }
  }
};  ///< END STRUCT ParameterGroup

class ParameterGroups : public vector<ParameterGroup> {
public:
  Parameters* operator[](const string& groupName) {
    for (auto& paramGroup : *this) {
      if (paramGroup.groupName == groupName) {
        return &paramGroup.params;
      }
    }
    return nullptr;
  }
};

/**
 * Class TComParamParser
 * command line parser
 */

class TComParamParser {
private:
  ConfigParameter m_configParam;   ///< configure file parameter
  ParameterGroups m_paramGroups;   ///< generic parameter group
  vector<string> m_invalidParams;  ///< invalid parameters
  Parameters* m_curParamGroup;     ///< a pointer to current active parameter group

public:
  TComParamParser()
    : m_curParamGroup(nullptr){};
  ~TComParamParser();

  void initParameters();
  void printHelp(ostream& out);
  void printParameters(ostream& out);
  void printInvalidParameters(ostream& out);

  void parseParameters(const int argc, const char* argv[]);
  void parseConfigureFile(const string& configFileName);

  TComParamParser& addParameter();

  ///< set configure file parameter
  TComParamParser& operator()(ConfigParserFunc* parser, const string nameShort,
                              const string nameLong, const string prompt) {
    m_configParam = ConfigParameter(parser, nameShort, nameLong, prompt);
    return *this;
  }

  ///< set generic parameters
  template<typename T>
  TComParamParser& operator()(T& data, const T defVal, const string nameShort,
                              const string nameLong, const string prompt) {
    if (m_curParamGroup == nullptr) {
      m_paramGroups.push_back(ParameterGroup(""));
      m_curParamGroup = m_paramGroups[""];
    }
    m_curParamGroup->push_back(new Parameter<T>(data, defVal, nameShort, nameLong, prompt));
    return *this;
  }

  ///< create a new group of parameters
  TComParamParser& operator()(const string groupName) {
    if (m_paramGroups[groupName] == nullptr) {
      m_paramGroups.push_back(ParameterGroup(groupName));
    }
    m_curParamGroup = m_paramGroups[groupName];
    return *this;
  }

private:
  bool setLongParameter(const string& key, const string& val);
  bool setShortParameter(const string& key, const string& val);
  int getKey(int& argi, const char* argv[], string& key);
  void getValue(int& argi, const char* argv[], string& val);

};  ///< END CLASS TComParamParser

//////////////////////////////////////////////////////////////////////////
// static inline functions
//////////////////////////////////////////////////////////////////////////

static inline void ConfigParser(const string& configFileName, vector<string>& keys,
                                vector<string>& vals) {
  keys.clear();
  vals.clear();

  ifstream file(configFileName, ifstream::in);

  if (!file) {  ///< fail to open configure file
    cerr << "Error: fail to load configure file " << configFileName << endl;
    return;
  }

  string line, token;
  size_t startPos, endPos;
  while (!file.eof()) {
    getline(file, line);

    startPos = line.find_first_not_of(" \t\n\r");
    endPos = line.find_first_of("#");
    if (startPos == string::npos || startPos == endPos)
      continue;  ///< skip empty line or comment line

    line = line.substr(startPos, endPos - startPos);  ///< remove comments

    vector<string> tokens;
    istringstream ss(line);
    while (getline(ss, token, ':'))  ///< read tokens from the line split by ':'
    {
      token.erase(remove_if(token.begin(), token.end(), [](char x) { return std::isspace(x); }),
                  token.end());  ///< remove whitespace
      if (token.length())
        tokens.push_back(token);
    }

    if (tokens.size() != 2)  ///< valid parameter should have exact 2 tokens
      continue;

    ///< set parameters
    keys.push_back(tokens[0]);
    vals.push_back(tokens[1]);
  }
}

///< \}

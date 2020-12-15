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

#include "TComParamParser.h"
#include "contributors.h"
#include <iostream>

///< \in TLibCommon \{

//////////////////////////////////////////////////////////////////////////
// Public class functions
//////////////////////////////////////////////////////////////////////////

TComParamParser::~TComParamParser() {
  for (auto paraGroup : m_paramGroups) {
    for (auto para : paraGroup.params) {
      if (para)
        delete para;
      para = nullptr;
    }
  }
}

void TComParamParser::parseConfigureFile(const string& configFileName) {
  vector<string> keys;
  vector<string> vals;

  m_configParam.configParser(configFileName, keys, vals);

  for (int i = 0; i < keys.size(); i++) {
    bool findKey = setLongParameter(keys[i], vals[i]);
    if (!findKey)
      findKey = setShortParameter(keys[i], vals[i]);
    if (!findKey)
      m_invalidParams.push_back(keys[i]);
  }
}

void TComParamParser::initParameters() {
  for (const auto& paraGroup : m_paramGroups) {
    for (const auto& para : paraGroup.params)
      para->init();
  }
}

void TComParamParser::printHelp(ostream& out) {
  ///< get the max length of long parameters and short parameters
  int maxLengthLong = 0;
  int maxLengthShort = 0;
  for (const auto& paraGroup : m_paramGroups) {
    for (const auto& para : paraGroup.params) {
      if (para->nameLong.length() > maxLengthLong)
        maxLengthLong = (int)para->nameLong.length();
      if (para->nameShort.length() > maxLengthShort)
        maxLengthShort = (int)para->nameShort.length();
    }
  }

  ///< print help infor
  for (auto& paraGroup : m_paramGroups) {
    paraGroup.printGroupName(out);
    for (const auto& para : paraGroup.params)
      para->printHelp(out, maxLengthLong, maxLengthShort);
  }
  out << endl;
}

void TComParamParser::printParameters(ostream& out) {
  ///< get the max length of long parameters
  int maxLengthLong = 0;
  for (const auto& paraGroup : m_paramGroups) {
    for (const auto& para : paraGroup.params)
      if (para->nameLong.length() > maxLengthLong)
        maxLengthLong = (int)para->nameLong.length();
  }
  for (auto& paraGroup : m_paramGroups) {
    paraGroup.printGroupName(out);
    for (const auto& para : paraGroup.params)
      para->printValue(out, maxLengthLong);
  }
  out << endl;
}

void TComParamParser::printInvalidParameters(ostream& out) {
  for (const auto& para : m_invalidParams) {
    out << "Warning: invalid parameter --" << para << endl;
  }
}

void TComParamParser::parseParameters(const int argc, const char* argv[]) {
  int idx = 1;
  string key, val;
  while (idx < argc && argv[idx] != nullptr) {
    switch (getKey(idx, argv, key)) {
    case KT_INVALID:  ///< invalid option
      m_invalidParams.push_back(key);
    case KT_LONG:  ///< double dash parameter, such as --help
      getValue(idx, argv, val);
      if (!setLongParameter(key, val))
        m_invalidParams.push_back(key);
      break;
    case KT_SHORT:  ///< single dash parameter, such as -h
      getValue(idx, argv, val);
      if (!setShortParameter(key, val))
        m_invalidParams.push_back(key);
      break;
    case KT_FILE:  ///< parse from configure file
      getValue(idx, argv, val);
      parseConfigureFile(val);
      break;
    default:
      break;
    }
  }
}

TComParamParser& TComParamParser::addParameter() {
  return *this;
}

//////////////////////////////////////////////////////////////////////////
// Private class functions
//////////////////////////////////////////////////////////////////////////

int TComParamParser::getKey(int& argi, const char* argv[], string& key) {
  string str(argv[argi++]);

  if (str.length() <= 1 || str[0] != '-')  ///< invalid key
  {
    key = str;
    return KT_INVALID;
  }

  ///< obtain key from the following pattern
  ///< --key
  ///< -key
  ///< --key=v
  ///< -key=v
  size_t startPos = str.find_first_not_of('-');
  size_t endPos = str.find_first_of('=');
  key = str.substr(startPos, endPos - startPos);

  if (key == m_configParam.nameLong || key == m_configParam.nameShort)
    return KT_FILE;

  return (str[1] == '-') ? KT_LONG : KT_SHORT;
}

void TComParamParser::getValue(int& argi, const char* argv[], string& val) {
  if (argv[argi] == nullptr)  ///< reach the last parameter
  {
    val = "1";  ///< infer default value is 1 (caution: only works for boolean parameters)
    return;
  }
  string str(argv[argi++]);

  if (str.length() == 0 || str[0] == '-')  ///< invalid value
  {
    val = "1";  ///< infer default value is 1 (caution: only works for boolean parameters)
    argi--;
    return;
  }
  val = str;
}

bool TComParamParser::setLongParameter(const string& key, const string& val) {
  bool findKey = false;
  for (auto paraGroup : m_paramGroups) {
    for (auto para : paraGroup.params)
      if (para->nameLong == key) {
        para->set(val);
        findKey = true;
        break;
      }
  }
  return findKey;
}

bool TComParamParser::setShortParameter(const string& key, const string& val) {
  bool findKey = false;
  for (auto paraGroup : m_paramGroups) {
    for (auto para : paraGroup.params)
      if (para->nameShort == key) {
        para->set(val);
        findKey = true;
        break;
      }
  }
  return findKey;
}

///< \}

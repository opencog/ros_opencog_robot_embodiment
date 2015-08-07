/*
 *   Config class for La Cogita IRC chatbot
 *   Copyright (C) 2009 Joel Pitt <joel@fruitionnz.com>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include "cog_irc/CogitaConfig.h"

#include <getopt.h>

#include <string>
#include <vector>
#include <iostream>
#include <stdlib.h>

//#include <opencog/util/StringTokenizer.h>

using std::cout;
using std::string;

using namespace std;

namespace opencog {
namespace chatbot {

CogitaConfig::CogitaConfig(std::string nick_name) :
    ircNetwork(COGITA_DEFAULT_SERVER), ircPort(COGITA_DEFAULT_PORT),
    vstring(COGITA_VSTRING), nick(nick_name)
{
        const char* defaultAttns[] = COGITA_DEFAULT_ATTN;
        const char* defaultSuffixes[] = COGITA_DEFAULT_ATTN_SUFFIXES;
        const char* defaultChannels[] = COGITA_DEFAULT_CHANNELS;
        createAttnVector();
        for (int i = 0; defaultChannels[i]; i++) {
            ircChannels.push_back(std::string(defaultChannels[i]));
        }

}

void CogitaConfig::createAttnVector()
{
    const char* defaultSuffixes[] = COGITA_DEFAULT_ATTN_SUFFIXES;
    attn.clear();
    for (int i = 0; defaultSuffixes[i]; i++) {
        attn.push_back(nick + string(defaultSuffixes[i]));
    }

}

}} // ~namespace opencog::chatbot

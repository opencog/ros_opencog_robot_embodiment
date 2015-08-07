/*
 *   IRC mod of La Cogita IRC chatbot
 *   Copyright (C) 2007 Linas Vepstas <linas@linas.org>
 *   Mod made by : Mandeep Singh Bhatia (30 July 2015)
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

/*
 * Go onto IRC
 * This is pretty totally a pure hack with little/no design to it.
 * Linas October 2007
 */

#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>
#include <getopt.h>

#include <string>
#include <vector>
#include <set>

#include "cog_irc/IRC.h"
#include "cog_irc/CogitaConfig.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "cog_irc/irc_msg.h"
//#include "whirr-sockets.h"
using namespace cog_irc;
using namespace opencog::chatbot;
using std::string;

CogitaConfig* cc;
ros::Publisher chatter_pub;
IRC conn;
/* printf can puke if these fields are NULL */
void fixup_reply(irc_reply_data* ird)
{
	ird->nick = (NULL == ird->nick) ? (char *) "" : ird->nick;
	ird->ident = (NULL == ird->ident) ? (char *) "" : ird->ident;
	ird->host = (NULL == ird->host) ? (char *) "" : ird->host;
	ird->target = (NULL == ird->target) ? (char *) "" : ird->target;
}

/**
 * Join channel shortly after logging into the irc server.
 */
int end_of_motd(const char* params, irc_reply_data* ird, void* data)
{
	IRC* conn = (IRC*) data;
	fixup_reply(ird);

	printf("chatbot got params=%s\n", params);
	printf("chatbot got motd nick=%s ident=%s host=%s target=%s\n",
		ird->nick, ird->ident, ird->host, ird->target);

	sleep(1);
	conn->join (cc->ircChannels[0].c_str());
	printf("chatbot sent channel join %s\n", cc->ircChannels[0].c_str());
	sleep(2);
	conn->notice (cc->ircChannels[0].c_str(), "ola");
	printf("chatbot sent channel notice\n");
	sleep(2);
	conn->privmsg (cc->ircChannels[0].c_str(), "here we are");
	printf("chatbot said hello to the channel\n");
	return 0;
}

/* return true if not all whitespace */
static bool is_nonblank(const char * str)
{
	size_t len = strlen(str);
	if (0 == len) return false;
	size_t blanks = strspn(str, " \n\r\t\v");
	if (blanks == len) return false;
	return true;
}

/**
 * Handle a message received from IRC.
 */
int got_privmsg(const char* params, irc_reply_data* ird, void* data)
{
	IRC* conn = (IRC*) data;
	fixup_reply(ird);

	printf("input=%s\n", params);
	printf("nick=%s ident=%s host=%s target=%s\n", ird->nick, ird->ident, ird->host, ird->target);

	typedef enum {ENGLISH=1, SHELL_CMD, SCM_CMD} CmdType;
	CmdType cmd = ENGLISH;

	const char * start = NULL;
	int priv = 0;
	if (!strcmp (ird->target, cc->nick.c_str())) {priv = 1; start = params+1; }

	if (!strncmp (&params[1], cc->nick.c_str(), cc->nick.size())) {
		start = params+1 + cc->nick.size();
		start = strchr(start, ':');
		if (start) start ++;
		////////////////
		///////////////////
	} else {
		// Check for alternative nick/attention strings
		for (string it :  cc->attn) {
			if (! it.compare(0,it.size(),&params[1]) ) {
				start = params + it.size();
				start = strchr(start, ':');
				if (start) start ++;
				break;
			}
		}
	}

	if (!start) return 0;
	char * msg_target = NULL;
//	if (priv)
//	{
		msg_target = ird->nick;
//	}
//	else
//	{
//		msg_target = ird->target;
//	}
/////////////////////////////////////////////
	// Reply to request for chat client version
	if ((0x1 == start[0]) && !strncmp (&start[1], "VERSION", 7))
	{
		printf ("VERSION: %s\n", cc->vstring.c_str());
		conn->privmsg (msg_target, cc->vstring.c_str());
		return 0;
	}
//start is the message
	// printf ("duude starting with 0x%x %s\n", start[0], start);
	//size_t textlen = strlen(start);
	irc_msg i_msg;
	i_msg.nick=msg_target;
	i_msg.message=start;
  chatter_pub.publish(i_msg);


	return 0;
}
void chatterCallback(const irc_msg::ConstPtr& msg)
{
	/*
	#define FLOOD_CHAR_COUNT 120

		size_t flood_cnt = FLOOD_CHAR_COUNT;
		size_t cnt = 0;
		bool dosend = true;
*/
		// printf ("Sending to opencog: %s\n", cmdline);
		  //ROS_INFO("I heard: [%s]", msg->message.c_str());

		string reply = msg->message;
		reply=msg->nick+": "+reply;
		const char* msg_target="#opencog";
		if (reply.length()<1 || reply.length()>300)return;
		conn.privmsg(msg_target,reply.c_str());
		
	////////////
		/* Each newline has to be on its own line */
		/* Limit length of reply so we don't get kicked for flooding */
/*
		const char * p = reply.c_str();
		while (*p)
		{
			const char *ep = strchr (p, '\n');

			// The last line -- no newline found.
			if (!ep)
			{
				if (is_nonblank(p))
					conn.privmsg (msg_target, p);
				break;
			}
			ep ++;
			//int save = *ep;
			//*ep = 0x0;

	////////////
			// If the line starts with ":scm", resubmit it to the
			// server. This is a kind-of cheap, hacky way of doing
			// multi-processing.
			// Else send output to chatroom
			if (dosend && is_nonblank(p))
			{
				conn.privmsg (msg_target, p);
				cnt += strlen (p);

				// Sleep so that we don't get kicked for flooding
				if (flood_cnt < cnt) { sleep(1); cnt -= FLOOD_CHAR_COUNT; }
				if (50 < flood_cnt) flood_cnt -= 15;
			}
			//*ep = save;
			p = ep;
		}
*/
	////////////////////////////////////////////////////////

}
int got_kick(const char* params, irc_reply_data* ird, void* data)
{
	fixup_reply(ird);
	printf("got kicked -- input=%s\n", params);
	printf("nick=%s ident=%s host=%s target=%s\n", ird->nick, ird->ident, ird->host, ird->target);
	return 0;
}

/**
 * @todo allow command line options via tclap http://tclap.sourceforge.net/ -
 * package libtclap-dev in Ubuntu.
 * However, its probably more portable to use plain-old getopt,
 * or maybe getopt_long, lets keep the dependency list minimal.
 * @todo use Config class to store defaults, and retrieve opencog.conf vars.
 */
int main (int argc, char * argv[])
{
	//whirr_sock_setup();
  int i=0;

  ros::init(argc, argv, "irc_talker");
	ros::NodeHandle n;
	chatter_pub = n.advertise<irc_msg>("irc_recv", 50);
	//if (cc.parseOptions(argc,argv)) return 0;
	std::string nick_name="ros_irc";
	/*
    if (n.getParam("/irc_nick_name", nick_name))
    {

    }
		*/
  cc=new CogitaConfig(nick_name);
	conn.hook_irc_command("376", &end_of_motd);
	conn.hook_irc_command("PRIVMSG", &got_privmsg);
	conn.hook_irc_command("KICK", &got_kick);

	const char *login = getlogin();

	// The login-name, nick, etc. are there only to make it look
	// pretty on IRC ident.
	conn.start (cc->ircNetwork.c_str(), cc->ircPort, cc->nick.c_str(), login,
	            "La ROS OpenCog chatbot", "asdf");

	ros::Subscriber sub = n.subscribe("irc_send", 50, chatterCallback);
	ros::Rate loop_rate(100);

	while (ros::ok()){
		ros::spinOnce();
	  i=conn.message_loop_once();//replace here
		loop_rate.sleep();
  }
  delete cc;
	if (i)fprintf(stderr, "%s: Fatal Error: Remote side closed socket\n",
		argv[0]);

	return i;
}

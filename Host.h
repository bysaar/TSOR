/*
 * --------FINAL_PROJECT----------
 * Saar Ben-Yochana - *********
 * Efi Korenfeld - *********
 * -------------------------------
 */

#ifndef HOST_H_
#define HOST_H_
#include <stdio.h>
#include <string.h>
#include <omnetpp.h>
#include <vector>
#include <algorithm>
#include <iostream>
#include "Packet_m.h"
#include <map>

using namespace omnetpp;
using namespace std;

class Host : public cSimpleModule
{
  private:
    cMessage *generate = nullptr;           // Self-Message to generate Data messages
    pkt* data_message = nullptr;            // Pointer to store last data message received
    double V = 0;                           // Distance from destination
    int R;                                  // Reward
    int lack_recieved = 0;                  // To count lack received
    int msg_num = 1;                        // Data message sequence number
    int last_msg_stat=0;                    // To maintain last data message received
    int host_index;                         // Self index

    // Configuration parametrs:
    string source_path;                     // The path for the SOURCE module, used to send statistic signals
    int net_type;                           // Indicate which topology is running
    int TARGET;                             // Target node
    bool OPT;                               // Indicates which algorithm is used (true->OPT, else->TSOR)

    // Maintained maps:
    map<int, int> neighbor_gates;           // Maps neighbor id to the matching gate
    map<int, pair<int,int>> beta_params;    // Stores beta distribution params (alpha and beta) for each neighbor
    map<int, double> neighbors_distances;   // Stores reported distances of neighbors from the destination
    map<double, int> lack_distances;        // Used to choose the next hop, picks the neighbor who's lack was received and reported the lowest distance

    //Statistic collection:
    simsignal_t costs_Signal;               // Costs of messages
    simsignal_t rewards_Signal;             // Rewards of messages
    cOutVector distance_vector;             // Distances from SOURCE
    cOutVector sequences_vec;               // Sequence number of messages



  public:
    Host();
    virtual ~Host();

  protected:
    /**
     * Initialization function, all neighbor tables and algorithm
     * parameters are set according to TSOR initialization
     */
    virtual void initialize() override;

    /**
     * Generates and return a message of specific type and name,
     * sets all of it's fields.
     */
    virtual pkt* generateMessage(int type, int dest, string msg_name,int seq_num);

    /**
     * Handles all type of messages and performing the appropriate action-
     * if self message then creates new data message, if statistic message then
     * record statistics, otherwise calls the function to handle message of each type
     */
    virtual void handleMessage(cMessage *msg) override;

    /**
     * Broadcasts a message from all available ports
     */
    virtual void broadcast(pkt* msg);

    /**
     * Handles data messages - creates and sends lack,
     * if arrived at destination sends Ack
     */
    virtual void handleDataMsg(pkt* msg);

    /**
     * Handles Ack and Nack messages,
     */
    virtual void handleAckNackMsg(pkt* msg,string name);

    /**
     * Single-server queue with a given service time.
     */
    virtual void handleLackMsg(pkt* msg);
    /**
     * Single-server queue with a given service time.
     */
    virtual void handleLcfmMsg(pkt* msg);
    /**
     * Prints all the hosts tables: neighbor_distances,
     * beta_params and neighbor_gates
     */
    virtual void printTables();

};


#endif /* HOST_H_ */

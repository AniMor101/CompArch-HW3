/* 046267 Computer Architecture - Winter 20/21 - HW #3 */

#include <vector>
#include <algorithm>
#include "dflow_calc.h"

using namespace std;

/// Maximum number of registers in the architecture
constexpr auto MAX_REGS = 32;

class InstNode {
public:
    int opcode;
    int latency;
    int index;
    int dep1;
    int dep2;
    int depth;

    // Constructors:

    InstNode(int opcode = -1, int latency = -1, int index = -1, int dep1 = -1, int dep2 = -1, int depth = 0):
        opcode(opcode), latency(latency), index(index), dep1(dep1), dep2(dep2), depth(depth) {}
};

class DepGraph {
private:
    const unsigned int* opsLatency_;
    const InstInfo *progTrace_;
    unsigned int numOfInsts_;
    int depth_;
    vector<int> regs_last_mod_by_inst_;
    vector<InstNode> nodes_;

    void addNode(int inst_idx, InstInfo inst_info) {
        int opcode = inst_info.opcode;
        int latency = opsLatency_[opcode];
        int dep1 = regs_last_mod_by_inst_[inst_info.src1Idx];
        int dep2 = regs_last_mod_by_inst_[inst_info.src2Idx];
        
        int tmp_depth1 = (dep1 != -1) ? nodes_[dep1].latency + nodes_[dep1].depth : 0;
        int tmp_depth2 = (dep2 != -1) ? nodes_[dep2].latency + nodes_[dep2].depth : 0;
        int node_depth = max(tmp_depth1, tmp_depth2);

        InstNode inst_node = InstNode(opcode, latency, inst_idx, dep1, dep2, node_depth);
        nodes_[inst_idx] = inst_node;
        
        if (inst_node.depth + inst_node.latency > depth_) {
            depth_ = inst_node.depth + inst_node.latency;
        }
    }

public:
    // Constructors:

    DepGraph(const unsigned int opsLatency[], const InstInfo progTrace[], unsigned int numOfInsts) :
        opsLatency_(opsLatency), progTrace_(progTrace), numOfInsts_(numOfInsts), depth_(0) {
        
        nodes_.resize(numOfInsts);
        regs_last_mod_by_inst_.resize(MAX_REGS);
        
        for (int i = 0; i < MAX_REGS; i++) {
            regs_last_mod_by_inst_[i] = -1;
        }

        for (int inst_idx = 0; inst_idx < numOfInsts; inst_idx++) {
            addNode(inst_idx, progTrace[inst_idx]);
            int dst_reg = progTrace[inst_idx].dstIdx;
            regs_last_mod_by_inst_[dst_reg] = inst_idx;
        }
    }

    // Getters:

    int getInstDepth(int inst_idx) {
        return nodes_[inst_idx].depth;
    }

    pair<int, int> getInstDeps(int inst_idx) {
        int dep1 = nodes_[inst_idx].dep1;
        int dep2 = nodes_[inst_idx].dep2;
        return make_pair(dep1, dep2);
    }

    int getGraphDepth() {
        return depth_;
    }

    unsigned int getNumOfInsts() {
        return numOfInsts_;
    }
};

DepGraph* prog_gragh;

ProgCtx analyzeProg(const unsigned int opsLatency[], const InstInfo progTrace[], unsigned int numOfInsts) {
    if (numOfInsts == 0) {
        return PROG_CTX_NULL;
    }
    prog_gragh = new DepGraph(opsLatency, progTrace, numOfInsts);
    if (prog_gragh == NULL) {
        return PROG_CTX_NULL;
    }
    return (void*)prog_gragh;
}

void freeProgCtx(ProgCtx ctx) {
    delete (DepGraph*)ctx;
}

int getInstDepth(ProgCtx ctx, unsigned int theInst) {
    DepGraph* graph_p = (DepGraph*)ctx;
    if (theInst >= graph_p->getNumOfInsts()) return -1;
    return graph_p->getInstDepth((int)theInst);
}

int getInstDeps(ProgCtx ctx, unsigned int theInst, int* src1DepInst, int* src2DepInst) {
    DepGraph* graph_p = (DepGraph*)ctx;
    if (theInst >= graph_p->getNumOfInsts()) return -1;
    *src1DepInst = graph_p->getInstDeps((int)theInst).first;
    *src2DepInst = graph_p->getInstDeps((int)theInst).second;
    return 0;
}

int getProgDepth(ProgCtx ctx) {
    DepGraph* graph_p = (DepGraph*)ctx;
    return graph_p->getGraphDepth();
}



#include <g2o_handler.hpp>

using namespace std;
using namespace gtsam;

void gn_2d(string g2o_file, string output_file){
    std::vector<boost::optional<IndexedPose>> indexed_poses;
    std::vector<boost::optional<IndexedEdge>> indexed_edges;
    std::vector<SharedNoiseModel> noise_models;

    load_g2o_2D(g2o_file, indexed_poses, indexed_edges, noise_models);

    int num_poses = indexed_poses.size();
    int num_edges = indexed_edges.size();

    std::cout<<num_poses<<" poses loaded. "<<num_edges<<" edges loaded."<<std::endl;

    NonlinearFactorGraph graph;

    std::cout<<"Adding prior..."<<std::endl;
    noiseModel::Diagonal::shared_ptr priorModel = \
        noiseModel::Diagonal::Variances(Vector3(0.3, 0.3, 0.1));
    graph.add(PriorFactor<Pose2>(0, Pose2(0,0,0), priorModel));

    Values initial;

    for(int i=0; i<num_poses; ++i){
        initial.insert(indexed_poses[i]->first, indexed_poses[i]->second);
    }
    
    
    for(int i=0; i<num_edges; ++i){
        auto edge = indexed_edges[i];
        auto model = noise_models[i];
        Key id1, id2;
        std::tie(id1, id2) = edge->first;
        NonlinearFactor::shared_ptr factor(
            new BetweenFactor<Pose2>(id1,id2, edge->second, model));
        graph.push_back(factor);
        
        std::cout<<"adding edge #: "<<i<<std::endl;
    }
    GaussNewtonParams params;
    GaussNewtonOptimizer optimizer(graph, initial, params);
    initial = optimizer.optimize();
    auto result = initial;

    NonlinearFactorGraph::shared_ptr graphNoKernel;
    Values::shared_ptr initial2;
    boost::tie(graphNoKernel, initial2) = readG2o(g2o_file);
    writeG2o(*graphNoKernel, result, output_file);

    std::cout << "done! " << std::endl;
}

void isam_2d(string g2o_file, string output_file){
    std::vector<boost::optional<IndexedPose>> indexed_poses;
    std::vector<boost::optional<IndexedEdge>> indexed_edges;
    std::vector<SharedNoiseModel> noise_models;

    load_g2o_2D(g2o_file, indexed_poses, indexed_edges, noise_models);

    int num_poses = indexed_poses.size();
    int num_edges = indexed_edges.size();

    std::cout<<num_poses<<" poses loaded. "<<num_edges<<" edges loaded."<<std::endl;

    NonlinearFactorGraph graph;

    std::cout<<"Adding prior..."<<std::endl;
    noiseModel::Diagonal::shared_ptr priorModel = \
        noiseModel::Diagonal::Variances(Vector3(0.3, 0.3, 0.1));
    graph.add(PriorFactor<Pose2>(0, Pose2(0,0,0), priorModel));

    Values initial;
    initial.insert(indexed_poses[0]->first, indexed_poses[0]->second);

    ISAM2Params parameters;
    ISAM2 isam(parameters);
    isam.update(graph, initial);
    Values result = isam.calculateEstimate();
    graph.resize(0);
    initial.clear();

    for(int i=1; i<num_poses; ++i){
        initial.insert(indexed_poses[i]->first, result.at(indexed_poses[i-1]->first));
        
        for(int j=0; j<num_edges; ++j){
            auto edge = indexed_edges[j];
            if(edge->first.second==indexed_poses[i]->first){
                auto model = noise_models[j];
                Key id1, id2;
                std::tie(id1, id2) = edge->first;
                NonlinearFactor::shared_ptr factor(
                    new BetweenFactor<Pose2>(id1,id2, edge->second, model));
                graph.push_back(factor);
                
                std::cout<<"adding edge #: "<<i<<std::endl;
            }
        }
        isam.update(graph, initial);
        result = isam.calculateEstimate();

        graph.resize(0);
        initial.clear();
    }
    

    NonlinearFactorGraph::shared_ptr graphNoKernel;
    Values::shared_ptr initial2;
    boost::tie(graphNoKernel, initial2) = readG2o(g2o_file);
    writeG2o(*graphNoKernel, result, output_file);

    std::cout << "done! " << std::endl;
}

void gn_3d(string g2o_file, string output_file){
    const auto poses = parse3DPoses(g2o_file);
    const auto edges = parse3DFactors(g2o_file);

    NonlinearFactorGraph graph;

    std::cout<<"Adding prior..."<<std::endl;
    Key first_key = 0;
    noiseModel::Diagonal::shared_ptr priorModel \
        = noiseModel::Diagonal::Variances((Vector(6) << 1, 1, 1, 0.5, 0.5, 0.5).finished());
    graph.add(PriorFactor<Pose3>(0, poses.find(0)->second, priorModel));

    Values initial;

    for(const auto& pose : poses){
        initial.insert(pose.first, pose.second);
    }

    for(const auto& edge : edges){
        graph.push_back(edge);
    }

    GaussNewtonParams params;
    GaussNewtonOptimizer optimizer(graph, initial, params);
    initial = optimizer.optimize();
    auto result = initial;

    NonlinearFactorGraph::shared_ptr graphNoKernel;
    Values::shared_ptr initial2;
    boost::tie(graphNoKernel, initial2) = readG2o(g2o_file);
    writeG2o(*graphNoKernel, result, output_file);

    std::cout << "done! " << std::endl;
}

void isam_3d(string g2o_file, string output_file){
    const auto poses = parse3DPoses(g2o_file);
    const auto edges = parse3DFactors(g2o_file);

    NonlinearFactorGraph graph;

    std::cout<<"Adding prior..."<<std::endl;
    Key first_key = 0;
    noiseModel::Diagonal::shared_ptr priorModel \
        = noiseModel::Diagonal::Variances((Vector(6) << 1, 1, 1, 0.5, 0.5, 0.5).finished());
    graph.add(PriorFactor<Pose3>(0, poses.find(0)->second, priorModel));

    Values initial;
    initial.insert(poses.find(0)->first,  poses.find(0)->second);

    ISAM2Params parameters;
    ISAM2 isam(parameters);
    isam.update(graph, initial);
    Values result = isam.calculateEstimate();

    graph.resize(0);
    initial.clear();

    for(int i=1; i<poses.size(); ++i){
        initial.insert(poses.find(i)->first, result.at(poses.find(i-1)->first));
        for(int j=0; j<edges.size(); ++j){
            if(edges[j]->key2() == poses.find(i)->first){
                graph.push_back(edges[j]);
            }
        }

        isam.update(graph, initial);
        result = isam.calculateEstimate();

        graph.resize(0);
        initial.clear();
    }
    

    NonlinearFactorGraph::shared_ptr graphNoKernel;
    Values::shared_ptr initial2;
    boost::tie(graphNoKernel, initial2) = readG2o(g2o_file);
    writeG2o(*graphNoKernel, result, output_file);

    std::cout << "done! " << std::endl;
}

int main(const int argc, const char *argv[]) {
    
    int question_num = std::stoi(argv[1]);
    string g2o_file = argv[2];
    string output_file = argv[3];

    switch(question_num){
        case 1:
            gn_2d(g2o_file,output_file);
            break;
        case 2:
            isam_2d(g2o_file,output_file);
            break;
        case 3:
            gn_3d(g2o_file,output_file);
            break;
        case 4:
            isam_3d(g2o_file,output_file);
            break;
    }

    
    
}
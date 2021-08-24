#include "replanner_to_goal.h"

namespace pathplan
{
ReplannerToGoal::ReplannerToGoal(Eigen::VectorXd& current_configuration,
                                 PathPtr& current_path,
                                 double max_time,
                                 const TreeSolverPtr &solver,
                                 const MetricsPtr& metrics,
                                 const CollisionCheckerPtr& checker,
                                 const Eigen::VectorXd& lb,
                                 const Eigen::VectorXd& ub)
{
  current_configuration_ = current_configuration;
  current_path_ = current_path;

  solver_ = solver;
  metrics_ = metrics;
  checker_ = checker;
  lb_ = lb;
  ub_ = ub;

  max_time_ = max_time;
  success_ = false;
}

void ReplannerToGoal::startReplannedPathFromNewCurrentConf(Eigen::VectorXd &configuration)
{

  std::vector<pathplan::ConnectionPtr> path_connections;
  std::vector<pathplan::ConnectionPtr> connections;

  pathplan::PathPtr path = std::make_shared<pathplan::Path>(replanned_path_->getConnections(),metrics_,checker_);
  pathplan::NodePtr current_node = std::make_shared<pathplan::Node>(configuration);
  pathplan::NodePtr path_start = path->getConnections().front()->getParent();

  for(const Eigen::VectorXd wp:path->getWaypoints())
  {
    if(wp == configuration)
    {
      if(wp == path->getWaypoints().back()) assert(0);
      replanned_path_ = path->getSubpathFromNode(current_node);
      return;
    }
  }

  int idx_current_conf, idx_path_start;
  double abscissa_current_conf = current_path_->curvilinearAbscissaOfPoint(configuration,idx_current_conf);

  double abscissa_path_start = current_path_->curvilinearAbscissaOfPoint(path_start->getConfiguration(),idx_path_start);

  if(abscissa_current_conf == abscissa_path_start)
  {
    return;  //the start of the replanned path is the current configuration
  }
  else if(abscissa_current_conf < abscissa_path_start)  //the replanned path starts from a position after the current one
  {
    if(idx_current_conf == idx_path_start)
    {
      //Directly connect the current configuration with the start of the replanned path
      ConnectionPtr conn = std::make_shared<Connection>(current_node, path_start);
      double cost_conn = metrics_->cost(configuration,path_start->getConfiguration());
      conn->setCost(cost_conn);
      conn->add();

      connections.push_back(conn);
    }
    if(idx_current_conf < idx_path_start)
    {
      NodePtr child = current_path_->getConnections().at(idx_current_conf)->getChild();

      if(child->getConfiguration() != configuration)
      {
        ConnectionPtr conn = std::make_shared<Connection>(current_node, child);
        double cost_conn = metrics_->cost(configuration,child->getConfiguration());
        conn->setCost(cost_conn);
        conn->add();

        connections.push_back(conn);
      }

      //Adding the connections between the two configurations
      for(unsigned int z = idx_current_conf+1; z<idx_path_start; z++) connections.push_back(current_path_->getConnections().at(z));

      if(path_start->getConfiguration() == current_path_->getConnections().at(idx_path_start)->getChild()->getConfiguration())
      {
        connections.push_back(current_path_->getConnections().at(idx_path_start));
      }
      else
      {
        NodePtr parent = current_path_->getConnections().at(idx_path_start)->getParent();
        if(parent->getConfiguration() != path_start->getConfiguration())
        {
          ConnectionPtr conn = std::make_shared<Connection>(parent,path_start);
          double cost_conn = metrics_->cost(parent->getConfiguration(),path_start->getConfiguration());
          conn->setCost(cost_conn);
          conn->add();

          connections.push_back(conn);
        }
      }
    }

    for(const ConnectionPtr& replanned_connection:path->getConnections()) connections.push_back(replanned_connection);
    path->setConnections(connections);
    replanned_path_ = path;

    return;
  }
  else //the replanned path starts from a position before the current configuration
  {
    pathplan::NodePtr node;

    int idx_conn;
    if(path->findConnection(configuration,idx_conn) != NULL)
    {
      node = path->getConnections().at(idx_conn)->getChild();

      if((path->getConnections().size()-1) > idx_conn)
      {
        for(unsigned int i=idx_conn+1; i<path->getConnections().size();i++) path_connections.push_back(path->getConnections().at(i));
      }
      for(unsigned int i=0; i<=idx_conn; i++) path->getConnections().at(i)->remove();

      if(current_node->getConfiguration() != node->getConfiguration())
      {
        pathplan::ConnectionPtr conn = std::make_shared<pathplan::Connection>(current_node,node);
        double cost = metrics_->cost(current_node,node);
        conn->setCost(cost);
        conn->add();

        connections.push_back(conn);
      }

      if((path->getConnections().size()-1) > idx_conn) connections.insert(connections.end(),path_connections.begin(),path_connections.end());

      path->setConnections(connections);
    }
    else
    {
      ConnectionPtr conn = current_path_->getConnections().at(idx_path_start);

      int idx = idx_path_start;
      if(path_start->getConfiguration() == current_path_->getConnections().at(idx_path_start)->getChild()->getConfiguration()) idx = idx_path_start + 1;

      int j = 0;
      int j_save = -2;
      for(unsigned int i=idx; i<current_path_->getConnections().size();i++)
      {
        if(path->getConnections().at(j)->getChild()->getConfiguration() == current_path_->getConnections().at(i)->getChild()->getConfiguration())
        {
          node = path->getConnections().at(j)->getChild();
          j_save = j;
        }
        else break;
        j+=1;
      }

      bool add_conn = false;
      if(j_save != -2)
      {
        for(unsigned int i=j_save+1; i<path->getConnections().size();i++) path_connections.push_back(path->getConnections().at(i));
        for(unsigned int i=0; i<=j_save; i++) path->getConnections().at(i)->remove();
      }
      else
      {
        if((conn->getParent()->getConfiguration() == path_start->getConfiguration()) || (conn->getChild()->getConfiguration() == path_start->getConfiguration()))
        {
          node = path_start;
          path_connections = path->getConnections();
        }
        else
        {
          if(idx_current_conf == idx_path_start) node = current_node;
          else node = current_path_->getConnections().at(idx_path_start)->getChild();
          path_connections = path->getConnections();
          add_conn = true;
        }
      }

      bool connected = false;
      if(add_conn && idx_path_start == idx_current_conf) connected = true;  //if the current conf is near after the pat start, you only have to connect these two confs

      int t = idx_current_conf;
      pathplan::NodePtr child;
      pathplan::NodePtr parent;

      while(!connected)
      {
        if(t == idx_current_conf) child = current_node;
        else child = std::make_shared<pathplan::Node>(current_path_->getConnections().at(t)->getChild()->getConfiguration());

        parent = std::make_shared<pathplan::Node>(current_path_->getConnections().at(t)->getParent()->getConfiguration());

        if(child->getConfiguration() != node->getConfiguration())
        {
          pathplan::ConnectionPtr conn = std::make_shared<pathplan::Connection>(child,parent); //you re moving backwards
          double cost = metrics_->cost(child,parent);
          conn->setCost(cost);
          conn->add();

          connections.push_back(conn);
        }
        else connected = true;

        t-=1;
      }

      if(add_conn)
      {
        pathplan::ConnectionPtr conn = std::make_shared<pathplan::Connection>(node,path_start); //you are moving backwards
        double cost = metrics_->cost(node,path_start);
        conn->setCost(cost);
        conn->add();

        connections.push_back(conn);
      }
      connections.insert(connections.end(),path_connections.begin(),path_connections.end());
      path->setConnections(connections);
    }

    replanned_path_ = path;
  }
}

bool ReplannerToGoal::computeConnectingPath(const NodePtr &path1_node_fake,
                                            const NodePtr &path2_node_fake,
                                            const double &diff_subpath_cost,
                                            PathPtr &connecting_path,
                                            bool &directly_connected)
{
  SamplerPtr sampler = std::make_shared<InformedSampler>(path1_node_fake->getConfiguration(), path2_node_fake->getConfiguration(), lb_, ub_,diff_subpath_cost);

  solver_->setSampler(sampler);
  solver_->resetProblem();
  solver_->addStart(path1_node_fake);

  ros::WallTime tic_solver = ros::WallTime::now();
  solver_->addGoal(path2_node_fake,max_time_);
  ros::WallTime toc_solver = ros::WallTime::now();

  directly_connected = solver_->solved();
  bool solver_has_solved;

  if(directly_connected)
  {
    connecting_path = solver_->getSolution();
    solver_has_solved = true;
  }
  else
  {
    double solver_time = max_time_-(toc_solver-tic_solver).toSec();
    solver_has_solved = solver_->solve(connecting_path,10000,solver_time);
  }

  return solver_has_solved;
}

PathPtr ReplannerToGoal::concatConnectingPathAndSubpath2(const std::vector<ConnectionPtr>& connecting_path_conn,
                                                         const std::vector<ConnectionPtr>& subpath2,
                                                         const NodePtr& path1_node,
                                                         const NodePtr& path2_node)
{
  std::vector<ConnectionPtr> new_connecting_path_conn;

  if(connecting_path_conn.size()>1)
  {
    NodePtr node1 = connecting_path_conn.front()->getChild();
    NodePtr node2 = connecting_path_conn.back()->getParent();

    double conn1_cost = metrics_->cost(path1_node,node1);
    double conn2_cost = metrics_->cost(node2,path2_node);

    ConnectionPtr conn1 = std::make_shared<Connection>(path1_node,node1);
    ConnectionPtr conn2 = std::make_shared<Connection>(node2,path2_node);

    conn1->setCost(conn1_cost);
    conn2->setCost(conn2_cost);

    conn1->add();
    conn2->add();

    new_connecting_path_conn.push_back(conn1);
    if(connecting_path_conn.size()>2) new_connecting_path_conn.insert(new_connecting_path_conn.end(), connecting_path_conn.begin()+1, connecting_path_conn.end()-1);
    new_connecting_path_conn.push_back(conn2);

    connecting_path_conn.front()->remove();
    connecting_path_conn.back()->remove();
  }
  else
  {
    ConnectionPtr conn1 = std::make_shared<Connection>(path1_node,path2_node);
    double conn1_cost =  metrics_->cost(path1_node,path2_node);
    conn1->setCost(conn1_cost);
    conn1->add();

    new_connecting_path_conn.push_back(conn1);
    connecting_path_conn.front()->remove();
  }

  if(!subpath2.empty())
  {
    new_connecting_path_conn.insert(new_connecting_path_conn.end(),subpath2.begin(),subpath2.end());
  }

  return std::make_shared<Path>(new_connecting_path_conn, metrics_, checker_);
}

bool ReplannerToGoal::connect2goal(const PathPtr& current_path, const NodePtr& node)
{
  success_ = false;

  NodePtr goal = current_path->getConnections().back()->getChild();
  NodePtr node_fake = std::make_shared<Node>(node->getConfiguration());
  NodePtr goal_node_fake = std::make_shared<Node>(goal->getConfiguration());

  PathPtr connecting_path;
  bool directly_connected;

  bool solver_has_solved = computeConnectingPath(node_fake, goal_node_fake, std::numeric_limits<double>::infinity(), connecting_path,directly_connected);

  if (solver_has_solved)
  {
    ROS_INFO_STREAM("Solution found!");

    std::vector<ConnectionPtr>  connecting_path_conn = connecting_path->getConnections();
    std::vector<ConnectionPtr>  subpath2_conn;

    subpath2_conn.clear();
    if(!subpath2_conn.empty()) ROS_ERROR("Subpath2 not empty!");

    PathPtr new_path = concatConnectingPathAndSubpath2(connecting_path_conn,subpath2_conn,node,goal);

    mtx_.lock();
    replanned_path_ = new_path;
    success_ = true;
    mtx_.unlock();
  }
  else
  {
    ROS_ERROR("New path not found!");
  }

  //Regardless if you have found a solution or not, delete the fake nodes
  node_fake->disconnect();
  goal_node_fake->disconnect();

  return success_;
}


}

#include "replanners_lib/replanners/replanner_base.h"

namespace pathplan
{
ReplannerBase::ReplannerBase(const Eigen::VectorXd& current_configuration,
                             const PathPtr& current_path,
                             const double& max_time,
                             const TreeSolverPtr &solver)
{
  current_configuration_ = current_configuration;
  current_path_ = current_path;
  replanned_path_ = current_path;

  solver_  = solver;
  metrics_ = solver->getMetrics();
  checker_ = solver->getChecker();

  lb_ = solver->getSampler()->getLB();
  ub_ = solver->getSampler()->getUB();

  max_time_ = max_time;
  success_ = false;

  disp_ = NULL;
  verbose_ = false;
}

//void ReplannerBase::startReplannedPathFromNewCurrentConf(Eigen::VectorXd &configuration, const PathPtr& current_path)
void ReplannerBase::startReplannedPathFromNewCurrentConf(Eigen::VectorXd &configuration)
{
  std::vector<pathplan::ConnectionPtr> path_connections;
  pathplan::NodePtr replanned_path_start = replanned_path_->getConnections().front()->getParent();
  std::vector<ConnectionPtr> conn_replanned = replanned_path_->getConnections();

  //If the configuration matches to a node of the replanned path
  for(const Eigen::VectorXd wp:replanned_path_->getWaypoints())
  {
    if(wp == configuration)
    {
      if(wp == replanned_path_->getWaypoints().back())
        assert(0);

      replanned_path_ = replanned_path_->getSubpathFromNode(configuration);
      return;
    }
  }

  //Otherwise, if the configuration does not match to any path node..
  PathPtr path_conf2replanned;
  int idx_current_conf, idx_replanned_path_start;

  double abscissa_current_conf = current_path_->curvilinearAbscissaOfPoint(configuration,idx_current_conf);
  double abscissa_replanned_path_start = current_path_->curvilinearAbscissaOfPoint(replanned_path_start->getConfiguration(),idx_replanned_path_start);

  if(abscissa_current_conf == abscissa_replanned_path_start) //the replanned path starts from the current configuration (redundant check)
  {
    return;
  }
  else if(abscissa_current_conf < abscissa_replanned_path_start)  //the replanned path starts from a position after the current one
  {
    path_conf2replanned = current_path_->getSubpathToConf(replanned_path_start->getConfiguration(),true);
    path_conf2replanned = path_conf2replanned->getSubpathFromConf(configuration,true);

    path_connections = path_conf2replanned->getConnections();
    path_connections.insert(path_connections.end(),conn_replanned.begin(),conn_replanned.end());
  }
  else
  {
    int idx_current_conf_on_replanned;
    ConnectionPtr conn = replanned_path_->findConnection(configuration,idx_current_conf_on_replanned);

    if(conn)
      path_connections = replanned_path_->getSubpathFromConf(configuration,true)->getConnections();
    else
    {
      disp_->nextButton("CASO NO IN PATH");
      disp_->clearMarkers();
      path_conf2replanned = current_path_->getSubpathToConf(configuration,true);
      disp_->displayPath(path_conf2replanned);
      disp_->nextButton();
      disp_->clearMarkers();
      path_conf2replanned = path_conf2replanned->getSubpathFromConf(replanned_path_start->getConfiguration(),true);
      disp_->displayPath(path_conf2replanned);
      disp_->nextButton();

      path_conf2replanned->flip();

      std::vector<ConnectionPtr> conn_conf2replanned = path_conf2replanned->getConnections();

      //Delete redundant connections
      bool delete_conn = false;
      int conf2replanned_idx = 0;
      int replanned_idx = 0;

      for(unsigned int i=0;i<conn_conf2replanned.size();i++)
      {
        for(unsigned int j=0;j<conn_replanned.size();j++)
        {
          bool match1 = (conn_conf2replanned.at(i)->getParent()->getConfiguration() -conn_replanned.at(j)->getChild()->getConfiguration()).norm()<1e-06;
          bool match2 = (conn_conf2replanned.at(i)->getChild()->getConfiguration() - conn_replanned.at(j)->getParent()->getConfiguration()).norm()<1e-06;
          bool match3 = (conn_conf2replanned.at(i)->getParent()->getConfiguration() -conn_replanned.at(j)->getParent()->getConfiguration()).norm()<1e-06;
          bool match4 = (conn_conf2replanned.at(i)->getChild()->getConfiguration() - conn_replanned.at(j)->getChild()->getConfiguration()).norm()<1e-06;

          ROS_INFO("match1,2,3,4: %d,%d,%d,%d",match1,match2,match3,match4);

          if((match1 && match2) || (match3 && match4))
          {
            conf2replanned_idx = i;
            replanned_idx = j;
            delete_conn = true;

            ROS_INFO_STREAM("CONN DA ELIMINARE, I: "<<conf2replanned_idx<<" J: "<<replanned_idx);

            break;
          }
        }
        if(delete_conn)
          break;
      }

      if(delete_conn)
      {
        path_connections.insert(path_connections.end(),conn_conf2replanned.begin(),conn_conf2replanned.begin()+(conf2replanned_idx));
        path_connections.insert(path_connections.end(),conn_replanned.begin()+(replanned_idx+1),conn_replanned.end());
      }
      else
      {
        path_connections = conn_conf2replanned;
        path_connections.insert(path_connections.end(),conn_replanned.begin(),conn_replanned.end());
      }
    }
  }

  replanned_path_->setConnections(path_connections);

//  disp_->nextButton("0last");
//  disp_->clearMarkers();
//  disp_->displayPath(replanned_path_,99);
//  disp_->nextButton("last");

  return;

  //  std::vector<pathplan::ConnectionPtr> path_connections;
  //  std::vector<pathplan::ConnectionPtr> connections;

  //  pathplan::NodePtr current_node = std::make_shared<pathplan::Node>(configuration);
  //  pathplan::NodePtr replanned_path_start = replanned_path_->getConnections().front()->getParent();

  //  //If the configuration matches to a node of the replanned path
  //  for(const Eigen::VectorXd wp:replanned_path_->getWaypoints())
  //  {
  //    if(wp == configuration)
  //    {
  //      if(wp == replanned_path_->getWaypoints().back())
  //        assert(0);

  //      replanned_path_ = replanned_path_->getSubpathFromNode(configuration);
  //      return;
  //    }
  //  }

  //  //Otherwise, if the configuration does not match to any path node..
  //  int idx_current_conf, idx_replanned_path_start;
  //  double abscissa_current_conf = current_path_->curvilinearAbscissaOfPoint(configuration,idx_current_conf);
  //  double abscissa_replanned_path_start = current_path_->curvilinearAbscissaOfPoint(replanned_path_start->getConfiguration(),idx_replanned_path_start);

  //  if(abscissa_current_conf == abscissa_replanned_path_start) //the replanned path starts from the current configuration (redundant check)
  //  {
  //    return;
  //  }
  //  else if(abscissa_current_conf < abscissa_replanned_path_start)  //the replanned path starts from a position after the current one
  //  {
  //    if(idx_current_conf == idx_replanned_path_start)
  //    {
  //      //Directly connect the current configuration with the start of the replanned path
  //      ConnectionPtr conn = std::make_shared<Connection>(current_node, replanned_path_start);
  //      double cost_conn = metrics_->cost(configuration,replanned_path_start->getConfiguration());
  //      conn->setCost(cost_conn);
  //      conn->add();

  //      connections.push_back(conn);
  //    }
  //    else if(idx_current_conf < idx_replanned_path_start)
  //    {
  //      NodePtr child = current_path->getConnections().at(idx_current_conf)->getChild();

  //      if(child->getConfiguration() != configuration)
  //      {
  //        ConnectionPtr conn = std::make_shared<Connection>(current_node, child);
  //        double cost_conn = metrics_->cost(configuration,child->getConfiguration());
  //        conn->setCost(cost_conn);
  //        conn->add();

  //        connections.push_back(conn);
  //      }

  //      //Adding the connections between the two configurations
  //      for(unsigned int z = idx_current_conf+1; z<idx_replanned_path_start; z++)
  //        connections.push_back(current_path->getConnections().at(z));

  //      NodePtr parent = current_path->getConnections().at(idx_replanned_path_start)->getParent();
  //      if(parent->getConfiguration() != replanned_path_start->getConfiguration())
  //      {
  //        ConnectionPtr conn = std::make_shared<Connection>(parent,replanned_path_start);
  //        double cost_conn = metrics_->cost(parent->getConfiguration(),replanned_path_start->getConfiguration());
  //        conn->setCost(cost_conn);
  //        conn->add();

  //        connections.push_back(conn);
  //      }
  //      else
  //      {
  //        if(!connections.empty())
  //        {
  //          ConnectionPtr last_conn = connections.back();

  //          ConnectionPtr conn = std::make_shared<Connection>(last_conn->getParent(),replanned_path_start);
  //          double cost_conn = metrics_->cost(last_conn->getParent()->getConfiguration(),replanned_path_start->getConfiguration());
  //          conn->setCost(cost_conn);
  //          conn->add();

  //          last_conn->remove();
  //          connections.pop_back();
  //          connections.push_back(conn);
  //        }
  //      }
  //    }

  //    for(const ConnectionPtr& replanned_connection:replanned_path_->getConnections())
  //      connections.push_back(replanned_connection);

  //    replanned_path_->setConnections(connections);
  //    replanned_path_->cost();

  //    return;
  //  }
  //  else //the replanned path starts from a position before the current configuration
  //  {
  //    pathplan::NodePtr node;

  //    int idx_conn_on_replanned_path;
  //    if(replanned_path_->findConnection(configuration,idx_conn_on_replanned_path) != NULL)
  //    {
  //      node = replanned_path_->getConnections().at(idx_conn_on_replanned_path)->getChild();

  //      if((replanned_path_->getConnections().size()-1) > idx_conn_on_replanned_path)
  //      {
  //        for(unsigned int i=idx_conn_on_replanned_path+1; i<replanned_path_->getConnections().size();i++)
  //          path_connections.push_back(replanned_path_->getConnections().at(i));
  //      }
  //      for(unsigned int i=0; i<=idx_conn_on_replanned_path; i++)
  //        replanned_path_->getConnections().at(i)->remove(); // non dovrei se no rompo il tree

  //      if(current_node->getConfiguration() != node->getConfiguration())
  //      {
  //        pathplan::ConnectionPtr conn = std::make_shared<pathplan::Connection>(current_node,node);
  //        double cost = metrics_->cost(current_node,node);
  //        conn->setCost(cost);
  //        conn->add();

  //        connections.push_back(conn);
  //      }

  //      if((replanned_path_->getConnections().size()-1) > idx_conn_on_replanned_path)
  //        connections.insert(connections.end(),path_connections.begin(),path_connections.end());

  //      replanned_path_->setConnections(connections);
  //    }
  //    else
  //    {
  //      ConnectionPtr conn = current_path->getConnections().at(idx_replanned_path_start);

  //      int idx = idx_replanned_path_start;
  //      if(replanned_path_start->getConfiguration() == current_path->getConnections().at(idx_replanned_path_start)->getChild()->getConfiguration())
  //        idx = idx_replanned_path_start + 1;

  //      int j = 0;
  //      int j_save = -2;
  //      for(unsigned int i=idx; i<current_path->getConnections().size();i++)
  //      {
  //        if(replanned_path_->getConnections().at(j)->getChild()->getConfiguration() == current_path->getConnections().at(i)->getChild()->getConfiguration())
  //        {
  //          node = replanned_path_->getConnections().at(j)->getChild();
  //          j_save = j;
  //        }
  //        else break;
  //        j+=1;
  //      }

  //      bool add_conn = false;
  //      if(j_save != -2)
  //      {
  //        for(unsigned int i=j_save+1; i<replanned_path_->getConnections().size();i++)
  //          path_connections.push_back(replanned_path_->getConnections().at(i));

  //        for(unsigned int i=0; i<=j_save; i++)
  //          replanned_path_->getConnections().at(i)->remove(); // non dovrei se no rompo il tree
  //      }
  //      else
  //      {
  //        if((conn->getParent()->getConfiguration() == replanned_path_start->getConfiguration()) || (conn->getChild()->getConfiguration() == replanned_path_start->getConfiguration()))
  //        {
  //          node = replanned_path_start;
  //          path_connections = replanned_path_->getConnections();
  //        }
  //        else
  //        {
  //          if(idx_current_conf == idx_replanned_path_start)
  //            node = current_node;
  //          else
  //            node = current_path->getConnections().at(idx_replanned_path_start)->getChild();

  //          path_connections = replanned_path_->getConnections();
  //          add_conn = true;
  //        }
  //      }

  //      bool connected = false;
  //      if(add_conn && idx_replanned_path_start == idx_current_conf) connected = true;  //if the current conf is near after the pat start, you only have to connect these two confs

  //      int t = idx_current_conf;
  //      pathplan::NodePtr child;
  //      pathplan::NodePtr parent;

  //      while(!connected)
  //      {
  //        if(t == idx_current_conf) child = current_node;
  //        else child = std::make_shared<pathplan::Node>(current_path->getConnections().at(t)->getChild()->getConfiguration());

  //        parent = std::make_shared<pathplan::Node>(current_path->getConnections().at(t)->getParent()->getConfiguration());

  //        if(child->getConfiguration() != node->getConfiguration())
  //        {
  //          pathplan::ConnectionPtr conn = std::make_shared<pathplan::Connection>(child,parent); //you re moving backwards
  //          double cost = metrics_->cost(child,parent);
  //          conn->setCost(cost);
  //          conn->add();

  //          connections.push_back(conn);
  //        }
  //        else connected = true;

  //        t-=1;
  //      }

  //      if(add_conn)
  //      {
  //        pathplan::ConnectionPtr conn = std::make_shared<pathplan::Connection>(node,replanned_path_start); //you are moving backwards
  //        double cost = metrics_->cost(node,replanned_path_start);
  //        conn->setCost(cost);
  //        conn->add();

  //        connections.push_back(conn);
  //      }
  //      connections.insert(connections.end(),path_connections.begin(),path_connections.end());
  //      replanned_path_->setConnections(connections);
  //    }

  //    replanned_path_ = replanned_path_;
  //  }
}

bool ReplannerBase::computeConnectingPath(const NodePtr &path1_node_fake,
                                          const NodePtr &path2_node_fake,
                                          const double &diff_subpath_cost,
                                          PathPtr &connecting_path,
                                          bool &directly_connected)
{
  return computeConnectingPath(path1_node_fake,
                               path2_node_fake,
                               diff_subpath_cost,
                               connecting_path,
                               directly_connected,
                               solver_);
}

bool ReplannerBase::computeConnectingPath(const NodePtr &path1_node_fake,
                                          const NodePtr &path2_node_fake,
                                          const double &diff_subpath_cost,
                                          PathPtr &connecting_path,
                                          bool &directly_connected,
                                          TreeSolverPtr& solver)
{
  SamplerPtr sampler = std::make_shared<InformedSampler>(path1_node_fake->getConfiguration(), path2_node_fake->getConfiguration(), lb_, ub_,diff_subpath_cost);

  solver->setSampler(sampler);
  solver->resetProblem();
  solver->addStart(path1_node_fake);

  ros::WallTime tic_solver = ros::WallTime::now();
  solver->addGoal(path2_node_fake,max_time_);
  ros::WallTime toc_solver = ros::WallTime::now();

  directly_connected = solver->solved();
  bool solver_has_solved;

  if(directly_connected)
  {
    connecting_path = solver->getSolution();
    solver_has_solved = true;
  }
  else
  {
    double solver_time = max_time_-(toc_solver-tic_solver).toSec();
    solver_has_solved = solver->solve(connecting_path,10000,solver_time);
  }

  return solver_has_solved;
}

PathPtr ReplannerBase::concatConnectingPathAndSubpath2(const std::vector<ConnectionPtr>& connecting_path_conn,
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

}

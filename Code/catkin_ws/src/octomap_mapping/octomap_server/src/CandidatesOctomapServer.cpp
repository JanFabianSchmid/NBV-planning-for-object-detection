#include <octomap_server/OctomapServer.h>

using namespace octomap;
using octomap_msgs::Octomap;

namespace octomap_server
{

	void OctomapServer::processNewCandidate(const KeySet& occupied_cells, const PCLPointCloud& new_candidate)
	{
		uint new_candidate_label = new_candidate.points[0].r + new_candidate.points[0].g;
		std::map<uint, uint> label_overlaps;
    uint labelled_nodes = 0;

    ROS_INFO_STREAM("----------");
    ROS_INFO_STREAM("Candidate " << new_candidate_label);
    ROS_INFO_STREAM("Size: " << occupied_cells.size());

    computeOverlaps(occupied_cells, label_overlaps, labelled_nodes);

    ROS_INFO_STREAM("Total overlap with existing candidates is " << (double)labelled_nodes / occupied_cells.size() * 100 << " percent.");

    if ((double)labelled_nodes / occupied_cells.size() < 0.05)
    {
    	m_candidateList.addCandidate(new_candidate_label);
    	ROS_INFO_STREAM("New label " << new_candidate_label <<
    		" (" << (uint)m_candidateList.getColor(new_candidate_label).r << "," << (uint)m_candidateList.getColor(new_candidate_label).g << ")" << " created.");
    }
    else
    {
    	uint computed_candidate_label = computeLabel(label_overlaps);
    	ROS_INFO_STREAM(
	      "Candidate " << new_candidate_label << " will be merged into candidate " << computed_candidate_label <<
	      " (" << (uint)m_candidateList.getColor(computed_candidate_label).r << "," << (uint)m_candidateList.getColor(computed_candidate_label).g << ")");
    	new_candidate_label = computed_candidate_label;
    }
    
    // if (computed_label == 0)
    // {
    // 	ROS_INFO_STREAM("New label " << new_candidate_label << " will be created.");
    // 	m_candidateList.addCandidate(new_candidate_label);
    // }
    // else
    // {
    // 	new_candidate_label = computed_label;
    // }

    octomap::ColorOcTreeNode::Color color = m_candidateList.getColor(new_candidate_label);

    uint null_nodes = 0, non_null_nodes = 0;

    // Apply label
    for (KeySet::const_iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++)
    {
      ColorOcTreeNode* node = m_octree->search(*it, 0);

      m_octree->updateNode(*it, true);

      if (node != NULL)
      {
        octomap::ColorOcTreeNode::Color currentColor = node->getColor();

        if (color.r == currentColor.r && color.g == currentColor.g)
        {
          node->setColor(currentColor.r, currentColor.g, (uint)currentColor.b + 1);
        }
        else
        {
          if ((uint)currentColor.b > 1)
          {
            node->setColor(currentColor.r, currentColor.g, (uint)currentColor.b - 1);
          }
          else
          {
            node->setColor(color.r, color.g, 0);
          }
        }

        non_null_nodes++;
      }
      else
      {
        ColorOcTreeNode* node = m_octree->search(*it, 0);
        node->setColor(color);
        null_nodes++;
      }
    }

    ROS_INFO_STREAM("Null nodes: " << null_nodes << "; non-null nodes: " << non_null_nodes);

    fflush(stdout);
	}

	uint OctomapServer::computeLabel(const std::map<uint, uint>& labels)
	{
	  uint greatest_overlap_label = 0, greatest_overlap = 0, labelled_nodes = 0;

	  ROS_INFO_STREAM("Labels found: ");
    for (std::map<uint, uint>::const_iterator map_it = labels.begin(), map_end = labels.end(); map_it != map_end; map_it++)
    {
      ROS_INFO_STREAM(
        "Label: " << map_it->first <<
        " (" << (uint)m_candidateList.getColor(map_it->first).r << "," << (uint)m_candidateList.getColor(map_it->first).g << ")" <<
        "; count: " << map_it->second);

      if (map_it->first != octomap_server::UNLABELLED && map_it->second > greatest_overlap)
      {
        greatest_overlap = map_it->second;
        greatest_overlap_label = map_it->first;
      }
    }

	  return greatest_overlap_label;
	}

	void OctomapServer::computeOverlaps(const KeySet& occupied_cells, std::map<uint, uint>& labels, uint& labelled_nodes)
	{
	  labels[octomap_server::UNLABELLED] = 0;
	  
	  for (KeySet::const_iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++)
	  {
	    // Find what's at that cell already
	    ColorOcTreeNode* node = m_octree->search(*it, 0);

	    if (node != NULL)
	    {
	      // Check the label and increment appropriate counter
	      uint existing_label = m_candidateList.getLabel(node->getColor());

	      labels[existing_label]++;
	      
	      if (existing_label != octomap_server::UNLABELLED) labelled_nodes++;
	      // else labels[octomap_server::UNLABELLED]++;
	    }
	    else labels[octomap_server::UNLABELLED]++;
	  }
	}

	/**
	  Copied from https://github.com/OctoMap/octomap/issues/40. Author: GitHub user fmder.
	*/
	double OctomapServer::getNodeDepth(const octomap::OcTree *inOcTree, const octomap::OcTreeKey& inKey){
	  octomap::OcTreeNode* lOriginNode = inOcTree->search(inKey);
	  unsigned int lCount = 0;
	  int i = 1;

	  octomap::OcTreeKey lNextKey(inKey);
	  lNextKey[0] = inKey[0] + i;

	  // Count the number of time we fall on the same node in the positive x direction
	  while(inOcTree->search(lNextKey) == lOriginNode){
	      ++lCount;
	      lNextKey[0] = inKey[0] + ++i;
	  }

	  i = 1;
	  lNextKey[0] = inKey[0] - i;

	  // Count the number of time we fall on the same node in the negative x direction
	  while(inOcTree->search(lNextKey) == lOriginNode){
	      ++lCount;
	      lNextKey[0] = inKey[0] - ++i;
	  }

	  return inOcTree->getTreeDepth() - log2(lCount + 1);
	}

	bool OctomapServer::compareGroundTruthToCandidatesSrv(evaluation::CompareGroundTruthsToProposals::Request& req, evaluation::CompareGroundTruthsToProposals::Response& res)
	{
		ROS_INFO_STREAM("\n");
	  ROS_INFO_STREAM("In CompareGroundTruthsToCandidates service, ground truths given " << req.ground_truths.data.size());

	  //DEBUG visualisation
	  PCLPointCloud visualisation_pc;

	  uint counter = 0;

	  std::map<uint, double> candidate_sizes = computeAllCandidateSizes();

	  BOOST_FOREACH(sensor_msgs::PointCloud2& pc_msg, req.ground_truths.data)
	  {
	    counter++;

	    // ROS_INFO_STREAM("Ground truth in frame " << pc_msg.header.frame_id);
	    // ROS_INFO_STREAM("World frame: " << m_worldFrameId);

	    tf::StampedTransform sensorToWorldTf;
		  try {
		    m_tfListener.lookupTransform(m_worldFrameId, pc_msg.header.frame_id, pc_msg.header.stamp, sensorToWorldTf);
		  } catch(tf::TransformException& ex){
		    ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting evaluation");
		    fflush(stdout);
		    return false;
		  }

		  PCLPointCloud pcl_pc;
	    fromROSMsg(pc_msg, pcl_pc);

	    KeySet free_cells, occupied_cells;
	    unsigned char* colors = new unsigned char[3];

	    calculateFreeAndOccupiedCellsFromNonGround(pcl_pc, pointTfToOctomap(sensorToWorldTf.getOrigin()), colors, false, free_cells, occupied_cells);

	    std::map<uint, uint> candidate_overlaps;
	    uint total_labelled_nodes = 0;

	    ROS_INFO_STREAM("");
	    ROS_INFO_STREAM("Ground truth " << counter << ":");
	    ROS_INFO_STREAM("Size: " << occupied_cells.size());

	    computeOverlaps(occupied_cells, candidate_overlaps, total_labelled_nodes);

	    // ROS_INFO_STREAM("Total overlap with existing candidates is " << (double)total_labelled_nodes / occupied_cells.size() * 100 << " percent.");

	    uint proposal = computeLabel(candidate_overlaps);

	    if (proposal == octomap_server::UNLABELLED) ROS_INFO_STREAM("No suitable candidate found");
	    else ROS_INFO_STREAM("Proposal is candidate " << proposal);
	    
	    ROS_INFO_STREAM("Candidate size: " << candidate_sizes[proposal]);
	    ROS_INFO_STREAM("Overlap: " << candidate_overlaps[proposal]);

	    //DEBUG visualisation
	    visualisation_pc += pcl_pc;
	  }

	  //DEBUG visualisation
	  sensor_msgs::PointCloud2 visualisation_msg;
	  pcl::toROSMsg(visualisation_pc, visualisation_msg);
	  visualisation_msg.header.frame_id = m_worldFrameId;
	  m_visualisation_pub.publish(visualisation_msg);

	  return true;
	}

	std::map<uint, double> OctomapServer::computeAllCandidateSizes()
	{
	  std::map<uint, double> labels_counter;

	  for (OcTreeT::tree_iterator tree_it = m_octree->begin_tree(), tree_end = m_octree->end_tree(); tree_it != tree_end; tree_it++)
	  {
	    ColorOcTreeNode* leaf_node = m_octree->search(tree_it.getKey(), 0);
	    if (leaf_node != NULL) labels_counter[m_candidateList.getLabel(leaf_node->getColor())]++;
	  }

	  return labels_counter;
	}

}
COSC.69.13
22F
Andrew Chen

# PA4
## Method
This program performs multi robot task allocation (MRTA) for three robots using Sequential Auction. Each of the robots has a bidder node, and one of the robots has an auctioneer node. Each bidder node registers itself with the auctioneer node using the NewFollower service. As a result, the auctioneer holds references to all three robots.

There are three pose_setter nodes, one associated with each robot. Each pose_setter node sends a message to the one broadcaster node containing that robot's initial pose. The broadcaster broadcasts this initial pose as a tf transform associated with the robot.

Once all the bidders have been registered with the auctioneer node, the auctioneer node sends a message to the /sr Start Randomize topic, which the randomizer node is subscribed to. The randomizer node generates random waypoints within a certain range until it gets three waypoints that are spaced apart with a minimum distance of 1. The randomizer then publishes these waypoints as a Waypoints message to the /rw Random Waypoints topic, which the auctioneer is subscribed to. Once this happens, the auctioneer begins task allocation via sequential auction.

The auctioneer node sends to each bidder every waypoint as a Waypoints message via Auction actions. Each bidder makes a bid on each waypoint based on the cost formula 1/(distance + 1). The bids are then returned to the auctioneer via the Auction action. Once the auctioneer node receives all the bids, it performs sequential auction. While there remains unassigned waypoints or unassigned bidders, auctioneer finds the highest remaining bid and assigns the bidder to the waypoint as a winner. Each winning bidder is sent a waypoint to go to via the Allocate action. Once the bidder node receives this waypoint, it uses tf to convert this waypoint in world frame to the robot's frame. Then, it rotates in place and moves towards the converted waypoint.

## Evaluation
As shown in the video, this program works in the gazebo simulator. Bidders are correctly registered with the auctioneer node. Bidders will navigate to the waypoints for which they won the auction on and will move to those waypoints one bidder at a time. However, depending on the movement speed/distance and turning speed/distance, robots may drift away from the waypoint coordinates. Sensible weights have been set in bidder nodes, and the program performs well with these parameters.
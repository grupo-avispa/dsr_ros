# This message represents a Node in the DSR (Deep State Representation) graph.
# It contains metadata about the node, including its name, type, attributes, and status flags.
# The status flags indicate whether the node has been updated or deleted.

std_msgs/Header   header           # Header
string            name             # Name of the node
string            type             # Type of the node
string[]          attributes       # Attributes of the node
bool              updated          # True if the node has been updated
bool              deleted          # True if the node has been deleted

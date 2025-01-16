# This message represents a Edge in the DSR (Deep State Representation) graph.
# It contains metadata about the edge, including its parent, child, type, attributes, and status flags.
# The status flags indicate whether the edge has been updated or deleted.

std_msgs/Header   header           # Header
string            parent           # Parent node name
string            child            # Child node name
string            type             # Type of the node
string[]          attributes       # Attributes of the node
bool              deleted          # True if the node has been deleted
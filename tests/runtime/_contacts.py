"""Contact forces between the gripper links and an object, on both engines.

``ContactReader.read()`` returns ``{link name: Contact}`` for every gripper link touching the
object this frame: the points of ``tactile.contact_points.ContactPointReader`` (PhysX contact
report, Newton ``efc.force``) summed per link. The same approach as
grippers/demo/mcp_bridge/make_contact_plot.py.
"""

from __future__ import annotations

from tactile.contact_points import Contact, ContactPointReader, link_contacts

__all__ = ["Contact", "ContactReader"]


class ContactReader:
    def __init__(self, gripper: str, root: str, object_path: str, engine: str):
        self.points = ContactPointReader(gripper, root, engine, object_path)

    def prepare(self, stage) -> None:
        self.points.prepare(stage)

    def read(self) -> dict[str, Contact]:
        return link_contacts(self.points.read())

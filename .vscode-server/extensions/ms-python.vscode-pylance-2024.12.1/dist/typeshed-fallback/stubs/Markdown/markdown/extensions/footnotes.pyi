from collections import OrderedDict
from re import Pattern
from typing import ClassVar
from xml.etree.ElementTree import Element

from markdown.blockparser import BlockParser
from markdown.blockprocessors import BlockProcessor
from markdown.core import Markdown
from markdown.extensions import Extension
from markdown.inlinepatterns import InlineProcessor
from markdown.postprocessors import Postprocessor
from markdown.treeprocessors import Treeprocessor

FN_BACKLINK_TEXT: str
NBSP_PLACEHOLDER: str
RE_REF_ID: Pattern[str]

class FootnoteExtension(Extension):
    unique_prefix: int
    found_refs: dict[str, int]
    used_refs: set[str]
    def __init__(self, **kwargs) -> None: ...
    parser: BlockParser
    md: Markdown
    footnotes: OrderedDict[str, str]
    def reset(self) -> None: ...
    def unique_ref(self, reference: str, found: bool = False) -> str: ...
    def findFootnotesPlaceholder(self, root: Element) -> tuple[Element, Element, bool] | None: ...
    def setFootnote(self, id: str, text: str) -> None: ...
    def get_separator(self) -> str: ...
    def makeFootnoteId(self, id: str) -> str: ...
    def makeFootnoteRefId(self, id: str, found: bool = False) -> str: ...
    def makeFootnotesDiv(self, root: Element) -> Element | None: ...

class FootnoteBlockProcessor(BlockProcessor):
    RE: ClassVar[Pattern[str]]
    footnotes: FootnoteExtension
    def __init__(self, footnotes: FootnoteExtension) -> None: ...
    def detectTabbed(self, blocks: list[str]) -> list[str]: ...
    def detab(self, block: str) -> str: ...  # type: ignore[override]

class FootnoteInlineProcessor(InlineProcessor):
    footnotes: FootnoteExtension
    def __init__(self, pattern: str, footnotes: FootnoteExtension) -> None: ...

class FootnotePostTreeprocessor(Treeprocessor):
    footnotes: FootnoteExtension
    def __init__(self, footnotes: FootnoteExtension) -> None: ...
    def add_duplicates(self, li: Element, duplicates: int) -> None: ...
    def get_num_duplicates(self, li: Element) -> int: ...
    def handle_duplicates(self, parent: Element) -> None: ...
    offset: int

class FootnoteTreeprocessor(Treeprocessor):
    footnotes: FootnoteExtension
    def __init__(self, footnotes: FootnoteExtension) -> None: ...

class FootnotePostprocessor(Postprocessor):
    footnotes: FootnoteExtension
    def __init__(self, footnotes: FootnoteExtension) -> None: ...

def makeExtension(**kwargs) -> FootnoteExtension: ...
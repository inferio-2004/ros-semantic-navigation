#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
import rospy
from my_robot.srv import SemanticQuery, SemanticQueryResponse
from bs4 import BeautifulSoup
from rank_bm25 import BM25Okapi
from nltk.stem import WordNetLemmatizer
from nltk.tokenize import word_tokenize

# -----------------------
# Helpers
# -----------------------
lemmatizer = WordNetLemmatizer()

def normalize(text):
    """Lowercase, remove punctuation, tokenize, and lemmatize."""
    text = re.sub(r'[^a-z0-9\s]', ' ', text.lower())
    tokens = word_tokenize(text)
    return [lemmatizer.lemmatize(t) for t in tokens if t.strip()]

def normalize_set(text):
    return set(normalize(text))

# -----------------------
# Place model
# -----------------------
class Place:
    def __init__(self, name, x, y, yaw, tags, purpose, objects):
        self.name = name
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)
        self.tags = tags
        self.purpose = purpose
        self.objects = objects

    def tokens(self):
        bag = set()
        bag.update(normalize(self.name))
        bag.update(normalize(self.tags))
        bag.update(normalize(self.purpose))
        bag.update(normalize(self.objects))
        return bag

    def text(self):
        """Return concatenated text for BM25 indexing"""
        return f"{self.name} {self.tags} {self.purpose} {self.objects}"

# -----------------------
# Knowledge Base
# -----------------------
class HTMLKnowledgeBase:
    def __init__(self, path):
        self.path = path
        self.mtime = None
        self.places = []
        self.bm25 = None
        self._load()

    def _load(self):
        with open(self.path, "r", encoding="utf-8") as f:
            soup = BeautifulSoup(f.read(), "html.parser")
        rows = soup.find("table").find_all("tr")[1:]  # skip header
        places = []
        for r in rows:
            cols = [c.get_text(strip=True) for c in r.find_all("td")]
            if len(cols) >= 7:
                places.append(Place(*cols[:7]))
        self.places = places
        self.mtime = os.path.getmtime(self.path)

        # Build BM25 index
        tokenized = [normalize(p.text()) for p in self.places]
        self.bm25 = BM25Okapi(tokenized)

        rospy.loginfo("Loaded %d places from KB", len(self.places))

    def maybe_reload(self):
        if os.path.getmtime(self.path) > self.mtime:
            rospy.loginfo("Reloading KB")
            self._load()

# -----------------------
# Retrieval
# -----------------------
def explain(place, query_tokens):
    hits = place.tokens() & query_tokens
    return f"matched keywords: {', '.join(hits)} | purpose: {place.purpose}"

# -----------------------
# Service
# -----------------------
class SemanticServer:
    def __init__(self):
        kb_path = rospy.get_param("~kb_html_path")
        self.kb = HTMLKnowledgeBase(kb_path)
        self.service = rospy.Service("semantic_query", SemanticQuery, self.handle)

    def handle(self, req):
        query = req.query
        if not query.strip():
            return SemanticQueryResponse(False, "", 0.0, 0.0, 0.0, "Empty query")

        self.kb.maybe_reload()
        query_tokens = normalize(query)

        # Score places using BM25
        scores = self.kb.bm25.get_scores(query_tokens)
        best_index = scores.argmax()
        best_place = self.kb.places[best_index]

        # If BM25 score is 0, consider no match
        if scores[best_index] == 0:
            return SemanticQueryResponse(False, "", 0.0, 0.0, 0.0, "No match")

        return SemanticQueryResponse(
            True,
            best_place.name,
            best_place.x,
            best_place.y,
            best_place.yaw,
            explain(best_place, set(query_tokens))
        )

def main():
    rospy.init_node("semantic_nav_server")
    SemanticServer()
    rospy.spin()

if __name__ == "__main__":
    main()


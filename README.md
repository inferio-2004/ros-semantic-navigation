# ros-semantic-navigation
# 🤖 Semantic Navigation of a Robot in ROS Noetic

**Semantic navigation of a robot in ROS Noetic using symbolic reasoning and BM25-based knowledge retrieval.**

This project simulates a mobile robot in **ROS Noetic** capable of following **symbolic, human-like navigation commands** instead of raw coordinates.

---

## 📌 Overview
- Implements a **ROS service server–client system** where commands such as *“find a room with a chair”* are interpreted through a **semantic knowledge base (HTML)**.  
- Uses **BM25 ranking, tokenization, and lemmatization** to map abstract instructions to the most relevant environment location.  
- Grounds symbolic goals into **actionable coordinates** and integrates with the **ROS navigation stack** for path planning.  
- Demonstrates **symbolic grounding and reasoning in robotics**, a foundational step in **cognitive robotics research**.  

---

## 🛠 Tech Stack
- **ROS Noetic**  
- **Python**  
- **BeautifulSoup (bs4)**  
- **Rank-BM25**  
- **NLTK**  
- **HTML Knowledge Base**  
- **Linux (Ubuntu 20.04 recommended)**  

---

## 🚀 Implementation So Far
![WhatsApp Image 2025-09-05 at 13 15 14_3cba15e2](https://github.com/user-attachments/assets/c525c1ce-407e-4833-b6f4-b770c1647582)

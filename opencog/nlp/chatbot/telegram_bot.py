#!/usr/bin/env python
# -*- coding: utf-8 -*-


TOKEN = 'YOUR_TOKEN'  # don't tell anyone!

from opencog.scheme_wrapper import scheme_eval_as, scheme_eval

python_atomspace = AtomSpace()
scheme_eval(python_atomspace, "(use-modules (opencog) (opencog exec))")
atomspace = scheme_eval_as('(cog-atomspace)')
scheme_eval(atomspace, '(use-modules (ice-9 readline))')
scheme_eval(atomspace, '(use-modules (opencog cogserver))')
scheme_eval(atomspace, '(use-modules (opencog nlp))')
scheme_eval(atomspace, '(use-modules (opencog nlp chatbot))')
scheme_eval(atomspace, '(use-modules (opencog nlp relex2logic))')
scheme_eval(atomspace, '(load-r2l-rulebase)')
print ("starting cogserver...")
scheme_eval(atomspace, '(start-cogserver "../lib/opencog-chatbot.conf")')


import logging

from telegram.ext import Updater, CommandHandler, MessageHandler, Filters

# Enable logging
logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                    level=logging.INFO)

logger = logging.getLogger(__name__)


# Define a few command handlers.
# These usually take the two arguments: bot and update. 
# Error handlers also receive the raised TelegramError object in error.

def start(bot, update):
    """Send a message when the  /start command is issued."""
    bot.send_message(chat_id=update.message.chat_id, text='Hello!')


def help(bot, update):
    """Send a message when the  /help command is issued."""
    bot.send_message(chat_id=update.message.chat_id, text='Help!')


def echo(bot, update):
    """Echo the user message."""
    print ("Ok, we got message {}".format(update.message.text))
    reply = scheme_eval(atomspace, '(process-query "{}" "{}")'.format(update.message.from_user.first_name, update.message.text))
    print ("And now we have a reply {}".format(reply))
    reply_decoded = reply.decode("utf-8")
    print ("Decoding the reply: {}".format(reply_decoded))
    bot.send_message(chat_id=update.message.chat_id, text=reply_decoded)


def error(bot, update):
    """Log Errors caused by Updates."""
    logger.warning('Update "%s" caused error "%s"', bot, update.error)


def main():
    """Start the bot."""
    # Create the Updater and pass it your bot's token.
    
    updater = Updater(TOKEN)

    # Get the dispatcher to register handlers
    dp = updater.dispatcher

    # on different commands - answer in Telegram
    dp.add_handler(CommandHandler("start", start))
    dp.add_handler(CommandHandler("help", help))

    # on noncommand i.e message - echo the message on Telegram
    dp.add_handler(MessageHandler(Filters.text, echo))

    # log all errors
    dp.add_error_handler(error)

    # Start the Bot
    updater.start_polling()

    # Run the bot until you press Ctrl-C or the process receives SIGINT,
    # SIGTERM or SIGABRT. This should be used most of the time, since
    # start_polling() is non-blocking and will stop the bot gracefully.
    updater.idle()


if __name__ == '__main__':
    main()
